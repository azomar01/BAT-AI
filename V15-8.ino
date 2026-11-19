/*
   SMART PERSONAL GPS TRACKER
   V10 - VIBRATION ALTERNANCE + TEST CONTINU
   ESP8266 + GP02

   GP02:
   TX -> GPIO5  (RX ESP)
   RX -> GPIO15 (TX ESP)
   POWER -> GPIO4

   A7670E POWER -> GPIO12
   VIBRATION     -> GPIO14

   SPIFFS configuration
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <Updater.h>
#include <math.h>

// ============================================================
// OTA : validateur de flux déclaré avant toutes les fonctions.
// Ceci évite les problèmes de prototypes générés automatiquement
// par Arduino IDE avec OTAStreamValidator.
// ============================================================
struct OTAStreamValidator {
  // Validation volontairement simple et tolérante.
  // Pendant le téléchargement, aucune erreur de structure ne coupe la
  // réception UART. La décision est prise uniquement à la fin.
  uint32_t total = 0;
  bool totalKnown = false;
  uint32_t pos = 0;
  uint8_t firstByte = 0;
  bool firstByteSeen = false;
  String reason = "";

  bool feed(const uint8_t *data, size_t len) {
    if (!len) return true;
    if (!firstByteSeen) {
      firstByte = data[0];
      firstByteSeen = true;
    }
    pos += (uint32_t)len;
    // Toujours continuer la réception. Aucun validateur ne doit interrompre
    // un bloc HTTPREAD de 1024 octets et laisser des données binaires dans UART.
    return true;
  }

  bool finish(String &why) {
    if (!totalKnown) total = pos;

    // Taille minimale très souple : on évite les faux rejets sur des images
    // dont la structure interne varie selon le core/outil de compilation.
    if (total < 16) {
      why = "FIRMWARE TROP PETIT";
      return false;
    }

    // Pour un firmware ESP8266 classique, le premier octet doit être E9.
    // C'est volontairement le seul contrôle de format obligatoire.
    if (!firstByteSeen || firstByte != 0xE9) {
      why = "MAGIC ESP8266 INVALIDE";
      return false;
    }

    why = "";
    return true;
  }
};

#define NUMO_VERSION "V15.8"

// Journal détaillé désactivé pour réduire Flash/RAM.
#define addLog(...)

// Forward declaration for Arduino auto-generated prototypes.
typedef struct TcpLinkState TcpLinkState;
// Forward declaration required by Arduino auto-generated function prototypes.
typedef struct SleepRTCData SleepRTCData;

bool modemOTAActive = false;
String otaLastError = "";
const size_t MODEM_OTA_CHUNK = 1024;

// OTA 4G : fenêtre maximale pendant laquelle on peut attendre que de
// nouvelles données deviennent disponibles dans le cache HTTP du modem.
// IMPORTANT : ce n'est PAS une attente bloquante de 30 s avant le premier
// HTTPREAD. Le manuel A76XX recommande au contraire de lire rapidement
// lorsque le contenu dépasse le cache du modem.
#define OTA_MODEM_DOWNLOAD_WAIT_MS 30000UL

// Petite pause entre deux commandes HTTPREAD.
#define OTA_HTTPREAD_GAP_MS 100UL

// Fenêtre très courte pour récupérer le trailer HTTPREAD.
#define OTA_HTTPREAD_TRAILER_WAIT_MS 80UL

// RX UART plus large pour éviter qu'un bloc binaire HTTP de 1024 octets
// arrive plus vite que l'ESP8266 ne puisse le vider.
#define OTA_MODEM_RX_BUFFER_SIZE 2048
volatile uint8_t otaProgressPercent = 0;
String otaStatusMessage = "En attente du fichier firmware...";
bool otaWebStreamActive = false;
volatile bool otaWebRequestPending = false;
String otaWebRequestURL = "";
volatile bool otaRestartPending = false;
uint32_t otaRestartAt = 0;
uint32_t otaBytesReceived = 0;
uint32_t otaTotalBytes = 0;
String otaStage = "En attente";
volatile bool otaOTAInProgress = false;
String otaSource = "";
void otaWebLog(const String &text);

// Journaux détaillés désactivés dans cette version légère pour libérer la RAM.
// Les macros ignorent aussi leurs arguments : aucune String temporaire créée.
#define traceModemTX(...) do {} while (0)
#define traceModemRX(...) do {} while (0)
#define traceGPSTX(...)   do {} while (0)
#define traceOTATX(...)   do {} while (0)
#define traceOTARX(...)   do {} while (0)
// Fichier temporaire OTA : le firmware est d'abord accumulé et vérifié.
const char* OTA_TEMP_FILE = "/ota.tmp";
File otaUploadFile; // conservé pour compatibilité avec les autres fonctions
bool otaUploadFailed = false;
size_t otaUploadReceived = 0;
// Mise à jour locale : recevoir directement dans la partition OTA, sans /ota.tmp.
OTAStreamValidator otaLocalUploadValidator;
bool otaLocalDirectMode = false;
bool otaLocalUpdateStarted = false;

// Nettoyage des fichiers temporaires/cache uniquement.
// Les fichiers utiles (configuration, buffer de trames, position du buffer
// et demande de localisation/SMS) ne sont jamais supprimés ici.
void cleanupTemporaryFiles() {
  bool changed = false;
  if (SPIFFS.exists(OTA_TEMP_FILE)) {
    if (SPIFFS.remove(OTA_TEMP_FILE)) changed = true;
  }
  // Fichier de travail utilisé pendant la compactation du buffer.
  // S'il reste après une interruption, il n'est plus utile et peut occuper
  // une quantité importante d'espace.
  if (SPIFFS.exists("/buffer.tmp")) {
    if (SPIFFS.remove("/buffer.tmp")) changed = true;
  }

  if (changed) {
    // Réouvrir le FS pour actualiser proprement l'espace disponible après
    // suppression des fichiers temporaires.
    SPIFFS.end();
    SPIFFS.begin();
  }
}

// Base GitHub fixe : l'utilisateur saisit uniquement le nom du fichier .ino.
// Le programme ajoute automatiquement .bin pour obtenir le firmware.
const char* OTA_GITHUB_FIRMWARE_BASE =
  "https://raw.githubusercontent.com/azomar01/BAT-AI/main/";

// =========================
// HARDWARE
// =========================
#define GPS_RX       5
#define GPS_TX       15
#define GPS_POWER    4
#define MODEM_POWER  12 // conserve pour compatibilite avec le firmware existant
// MODEM_RESET absent sur cette carte : redémarrage par coupure MODEM_POWER.
#define VIBRATION    14
#define MODEM_BAUD   115200

// =========================
// WIFI
// =========================
String apSSID = "NUMOTRONIC-NUMO-";
const char* AP_PASS = "12345678";

ESP8266WebServer server(80);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

void otaWebLog(const String &text) {
  otaStatusMessage = text;
  if (otaWebStreamActive) {
    server.sendContent(text);
    server.sendContent("\n");
  }
}

// =========================
// CONFIGURATION
// (renommé Config -> TrackerConfig : corrige
//  l'erreur "reference to 'Config' is ambiguous"
//  provoquée par un conflit de nom avec une
//  autre librairie du projet)
// =========================
struct TrackerConfig {
  String deviceID = "NUMO-SMART-"+ String(NUMO_VERSION);
  String imei =  "" ;

  String apn = "apn.ma" ;

  String server1 = "gps-tracker.ma";
  uint16_t port1 = 10200;
  bool server1Enable = true;

  String server2 = "";
  uint16_t port2 = 0;
  bool server2Enable = false;

  // Intervalles standardisés et modifiables depuis CONFIG
  uint16_t gpsInterval = 1;            // lecture GPS (s)
  uint16_t positionInterval = 15;      // NORMAL + PERFORMANCE + buffer ECO (s)
  uint32_t longIntervalMin = 60;       // grand intervalle commun (minutes)
  uint16_t stopDelay = 60;             // arrêt complet (s)
  uint16_t turnAngle = 15;              // changement de cap (degrés)
  uint16_t heartbeatInterval = 60;    // heartbeat TCP (s)

  uint16_t gpsTimeout = 180;
  uint16_t motionConfirmTimeout = 180; // recherche FIX après vibration (s)
  uint16_t sendTimeout = 60;          // délai maximal d'une défaillance pendant l'envoi (s)
  uint8_t minSat = 4;

  uint16_t bufferMax = 2000;
  uint8_t recoveryRate = 1;             // éléments/s pendant récupération

  uint8_t energyMode = 5;              // 0 NORMAL, 1 ECO, 2 PERFORMANCE, 3 ULTRA ECO, 4 TEST, 5 PERMANENCE

  bool vibrationEnable = true;
  bool smsEnable = true;
  bool ledEnable = true;              // voyant de signalisation sur la sortie 2

  bool bufferEnable = true;
  uint8_t transmitMode = 0; // 0=PRIMARY/BACKUP, 1=DOUBLE
  float batteryCalibration = 1.000f;

  // Gestion énergétique V6 - batterie Li-ion 1S
  bool autoPowerSave = true;
  float batteryLowV = 3.80f;
  float batteryCriticalV = 3.70f;
  uint8_t batteryAction = 2; // 0=BIEN, 1=AVERTIR, 2=AVERTIR_ET_ARRETER
  uint16_t gpsIdleOffDelay = 60;       // s après arrêt confirmé
  uint16_t modemIdleOffDelay = 120;     // s après dernière transmission
  uint16_t vibrationHold = 8;           // ancien paramètre conservé
  uint16_t vibrationThreshold = 5;
  uint16_t vibrationDebounceMs = 20;
  uint16_t vibrationWatchSeconds = 10; // durée de surveillance vibration au réveil (1 à 3600 s)

  // SMS : jusqu'à 3 numéros autorisés
  String smsAdmin1 = "0607578845";
  String smsAdmin2 = "";
  String smsAdmin3 = "";
  String smsPassword = "123456";
  String smsPrivateCode = "739251";

  // SERVEUR CMD - MQTT pour les commandes distantes uniquement
  bool mqttEnable = true;
  String mqttServer = "broker.emqx.io";
  uint16_t mqttPort = 1883;
  String mqttUser = "";
  String mqttPassword = "";
  uint16_t mqttKeepAlive = 60;
  String mqttReplyNumber = ""; // numero SMS qui recoit les retours des commandes MQTT
};

TrackerConfig cfg;

// =========================
// SERVEUR CMD - MQTT
// =========================
bool mqttServiceStarted = false;
bool mqttConnected = false;
bool mqttSubscribed = false;
bool mqttConnecting = false;
bool mqttClientAcquired = false;
unsigned long mqttLastAttempt = 0;
unsigned long mqttLastRx = 0;
unsigned long mqttLastAck = 0;
String mqttLastCommand = "";
String mqttLastAckText = "";
String mqttRxTopic = "";
String mqttRxPayload = "";
bool mqttCommandContext = false;
bool remoteCommandContext = false; // commande authentifiee par MQTT/TCP
String mqttCommandResponse = "";
bool mqttAckPending = false;
String mqttPendingAck = "";
bool mqttResponsePending = false;
String mqttPendingResponse = "";
bool mqttCommandPending = false;
String mqttPendingCommand = ""; // conserve pour compatibilite; les CMD MQTT sont maintenant executees a la reception
uint8_t mqttRxStage = 0;
uint16_t mqttRxExpected = 0;

String mqttCmdTopic() {
  String id = cfg.imei;
  if (id.length() != 15) id = cfg.deviceID;
  return "NUMO/" + id + "/CMD";
}
String mqttAckTopic() {
  String id = cfg.imei;
  if (id.length() != 15) id = cfg.deviceID;
  return "NUMO/" + id + "/CMD";
}
void taskMQTT();
bool mqttPublish(const String &topic, const String &payload, uint8_t qos, bool retain);
bool mqttConnectAndSubscribe();
void mqttHandleCommand(const String &payload);
void mqttResetState();
void mqttStopService();

// =========================
// GPS DATA
// =========================
struct GPSData {

  bool fix = false;

  double lat = 0;
  double lon = 0;
  double altitude = 0;
  double speed = 0;
  double course = 0;

  uint8_t satellites = 0;
  uint8_t fixQuality = 0;
  char rmcStatus = 'V';

  String utc = "--:--:--";
  String date = "--/--/----";
  String rawTime = "";
  String rawDate = "";
  String latNmea = "";
  String lonNmea = "";
  char latHem = 'N';
  char lonHem = 'E';

  unsigned long lastFix = 0;
};

GPSData gps;

// Filtre GPS minimal : seules latitude, longitude et vitesse sont controlees.
// Aucune comparaison avec l'ancienne position : pas de rejet de saut GPS.


// =========================
// SYSTEM
// =========================
unsigned long bootTime = 0;
unsigned long lastGPS = 0;
unsigned long lastWeb = 0;

bool gpsPower = false;
bool modemPower = false;

// Batterie : pont 220K haut / 10K vers GND sur A0
const float BAT_TOP_R = 220000.0f;
const float BAT_BOTTOM_R = 10000.0f;
const float BAT_A0_FULL_SCALE = 1.000f; // ADC interne ESP8266-07S : 0..1.0 V
const float BAT_DEFAULT_CALIBRATION = 0.8483f;
const float BAT_EMPTY_V = 3.50f;
const float BAT_FULL_V = 4.20f;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
unsigned long lastBatteryRead = 0;
float batteryA0Voltage = 0.0f;
uint16_t batteryRawADC = 0;

// V6 - état énergétique / mouvement matériel
bool motionEvent = false;
unsigned long lastVibrationEvent = 0;
unsigned long lastModemActivityForPower = 0;
unsigned long lastGpsActivityForPower = 0;
bool batteryProtectionActive = false;
bool batteryAlertSent = false;
bool batteryShutdownWait = false;
unsigned long batteryShutdownStart = 0;

// =========================
// DEEP-SLEEP / WAKE LOGIC
// =========================
const uint16_t WAKE_VIBRATION_WINDOW_S = 10;
const float MOTION_SPEED_THRESHOLD_KMH = 6.0f;
unsigned long wakeStartedAt = 0;
unsigned long motionConfirmStartedAt = 0;
bool wakeVibrationWindow = true;
bool motionConfirmationActive = false;
uint16_t vibrationTransitions = 0;
uint8_t vibrationLastState = HIGH;
unsigned long vibrationLastEdgeAt = 0;
bool activeTracking = false;
bool sleepRequested = false;

// ============================================================
// PLANIFICATION DES RÉVEILS ÉCONOMIQUES
// 1 minute de veille profonde entre deux réveils.
// Un compteur conservé dans la mémoire RTC permet de compter les
// minutes même après un redémarrage provoqué par la sortie de veille.
// ============================================================
struct SleepRTCData {
  uint32_t magic;
  uint32_t nextWakeMinute;
  uint32_t checksum;
};

const uint32_t SLEEP_RTC_MAGIC = 0x4E554D31UL; // NUM1
uint32_t sleepWakeMinute = 0;
bool sleepRtcValid = false;
bool scheduledCommunicationWake = false;
bool scheduledGrandWake = false;

// PERMANENCE : GPS/ESP restent actifs; le modem passe en Software Sleep.
bool permanenceModemSleep = false;
bool permanenceWakePending = false;
unsigned long permanenceWakeStartedAt = 0;
unsigned long permanenceNextWakeAt = 0;
const unsigned long PERMANENCE_WAKE_SETTLE_MS = 350UL;
bool scheduledSmsWake = false;
unsigned long communicationWakeStartedAt = 0;
unsigned long communicationWakeWindowMs = 0;
const unsigned long GRAND_COMMUNICATION_WINDOW_MS = 120000UL; // 2 min minimum
const unsigned long SMS_COMMUNICATION_WINDOW_MS = 45000UL;    // contrôle SMS court

// Demande LOCALISATION conservée si le GPS n'a pas encore de position.
bool pendingLocationSMS = false;
String pendingLocationNumber = "";
bool pendingLocationViaMQTT = false;
bool pendingLocationSending = false;
unsigned long pendingLocationLastAttempt = 0;
const unsigned long PENDING_LOCATION_RETRY_MS = 30000UL;
const unsigned long LOCATION_COMMUNICATION_WINDOW_MS = 120000UL;

void enterConfiguredDeepSleep(const char* reason);
void initializeSleepSchedule();
void saveSleepRTC(uint32_t nextMinute);
void taskScheduledCommunication();
void taskPendingLocationSMS();
void savePendingLocationRequest(const String &number, bool viaMQTT = false);
void loadPendingLocationRequest();
void clearPendingLocationRequest();
void taskStatusLED();
void initLinks();



// Runtime/network statistics
bool networkRegistered = false;
// CEREG est la source de reference pour l enregistrement LTE A7670E.
// CGREG ne doit pas ecraser un etat LTE deja connu.
bool ceregStatusKnown = false;
bool packetServiceAttached = false;
int networkRSSI = -1;
String networkOperator = "--";
String networkType = "--";
bool simPresent = false;
bool simReady = false;
String simState = "UNKNOWN";
String modemState = "OFF";
bool tcpServiceOpen = false;
bool tcpNetOpening = false;
unsigned long lastNetOpenAttempt = 0;
unsigned long lastNetworkPoll = 0;
unsigned long lastSimPoll = 0;
const unsigned long SIM_STATUS_POLL_INTERVAL_MS = 10000UL; // verification automatique de la presence SIM
const unsigned long TCP_RECONNECT_INTERVAL_MS = 10000UL; // délai entre deux tentatives serveur
// Connexion TCP : +CIPOPEN: 1,0 / 2,0 confirme le socket TCP concerne.
// La preuve finale peut être obtenue par CIPSEND confirmé.
// ----------------------------------------------------------------
const unsigned long NET_REOPEN_INTERVAL_MS = 15000UL; // délai entre deux tentatives NETOPEN
const uint8_t TCP_FAILURE_TOLERANCE = 2; // nb d'echecs consecutifs tolérés avant recovery complet (NETCLOSE)
unsigned long lastMovement = 0;
bool moving = false;
bool stopConfirmed = true;
unsigned long longIntervalStart = 0;
unsigned long lastPositionEvent = 0;
unsigned long lastTestTx = 0;
double lastTurnCourse = -1;
bool shortPositionPending = false;
bool longPositionPending = false;
uint8_t shortEventMask = 0;
String recoveryFrame = "";
uint8_t longEventMask = 0;

// Gestion d'énergie du mode NORMAL.
// Récupération déclenchée en mouvement à partir de 100 éléments :
// elle continue jusqu'à vider le buffer.
// Récupération après arrêt : fenêtre maximale de 2 minutes.
bool normalBufferRecoveryActive = false;
bool normalPostStopRecoveryActive = false;
unsigned long normalPostStopRecoveryStart = 0;
const unsigned long NORMAL_POST_STOP_RECOVERY_MAX_MS = 120000UL;

// CHECK : une position GPS est transmise au serveur toutes les 30 s
// pendant la fenetre CHECK lorsqu'un FIX est disponible.
String checkPositionFrame = "";
uint8_t checkPositionTargetMask = 0;
uint8_t checkPositionConfirmedMask = 0;
unsigned long lastCheckPositionSend = 0;
const unsigned long CHECK_POSITION_INTERVAL_MS = 30000UL;

// Wi-Fi : bouton GPIO0 -> GND, pression longue de 4 s
// Une pression longue inverse l'état ON/OFF une seule fois par appui.
// Voyant de signalisation : sortie 2.
bool wifiEnabled = false;
bool wifiButtonWasDown = false;
bool wifiButtonActionDone = false;
unsigned long wifiButtonStart = 0;
unsigned long wifiStartedAt = 0;
const unsigned long WIFI_LONG_PRESS_MS = 4000;
const unsigned long WIFI_AUTO_OFF_MS = 300000UL; // 5 min
const uint8_t WIFI_BUTTON_PIN = 0;
const uint8_t STATUS_LED_PIN = 2;
const uint8_t STATUS_LED_ON_LEVEL = LOW;
const uint8_t STATUS_LED_OFF_LEVEL = HIGH;

// Store & Forward : fichiers segmentes de 100 positions maximum.
const uint16_t BUFFER_FILE_CAPACITY = 100;
const uint16_t BUFFER_MAX_FILE_INDEX = 9999;
uint16_t bufferFileIndex = 0;       // dernier fichier existant
uint16_t bufferFileCount = 0;       // positions dans le dernier fichier
uint16_t bufferOldestFileIndex = 0; // fichier FIFO actuellement lu
uint16_t bufferOldestFileCount = 0; // nombre total de lignes du fichier FIFO
uint16_t bufferOldestFileOffset = 0;// lignes deja confirmees dans ce fichier
uint32_t bufferOldestByteOffset = 0; // position octet de la prochaine trame FIFO
uint32_t bufferPeekEndOffset = 0; // fin de la trame actuellement en attente d'envoi
uint32_t bufferPending = 0;
uint32_t bufferSent = 0;
uint32_t bufferRetry = 0;
uint32_t bufferLost = 0;
const size_t BUFFER_RESERVED_SPACE = 16384; // marge SPIFFS de securite
unsigned long lastSend = 0;
unsigned long lastRecovery = 0;

struct TcpLinkState {
  bool enabled = false;
  bool connected = false;
  bool opening = false;
  bool sending = false;
  bool loginSent = false;
  bool waitingServerAck = false;
  uint8_t socket = 0;
  String host = "";
  uint16_t port = 0;
  unsigned long lastConnectAttempt = 0;
  unsigned long openingSince = 0;   // début d'attente CIPOPEN
  unsigned long lastSend = 0;
  unsigned long lastHeartbeat = 0;
  unsigned long lastRx = 0;
  unsigned long lastTxSuccessAt = 0;
  uint32_t txCount = 0;
  uint32_t reconnectCount = 0;
  uint8_t txFailureStreak = 0;
  uint8_t connectFailureStreak = 0; // tolérance avant recovery complet
};
TcpLinkState link1, link2;
String modemTcpPending = "";
uint8_t modemTcpPendingSocket = 255;
bool modemExpectSendPrompt = false;
bool modemTcpBusy = false;
unsigned long modemTcpStartedAt = 0;
unsigned long modemTcpDataSentAt = 0;
uint32_t tcpTxSuccess = 0;
uint32_t tcpTxFailure = 0;
const unsigned long TCP_TX_TIMEOUT_MS = 8000UL;
const unsigned long TCP_CIPOPEN_TIMEOUT_MS = 15000UL;
const unsigned long TCP_TX_MIN_GAP_MS = 50UL;
const unsigned long TCP_SEND_RETRY_INTERVAL_MS = 1000UL;
const uint8_t TCP_MAX_SEND_FAILURES_BEFORE_RECONNECT = 3;
const unsigned long TCP_STALE_CONNECTION_MS = 30000UL; // aucune confirmation TX avec buffer
unsigned long lastGlobalTcpTx = 0;
// 0=position, 1=heartbeat, 2=buffer
uint8_t modemTxPurpose = 0;
uint8_t modemRxSocket = 255;

// Envoi BUFFER base sur la logique historique sending():
// serveur 0 d'abord, puis serveur 1 si configuré.
// La trame reste dans SPIFFS tant que tous les serveurs requis ne sont pas confirmes.
uint8_t bufferTargetMask = 0;
uint8_t bufferConfirmedMask = 0;
uint8_t bufferActiveSocket = 255;

// OTA state
volatile size_t otaReceived = 0;
volatile size_t otaTotal = 0;
volatile bool otaRunning = false;
volatile bool otaSuccess = false;


// Derniere trame GPS103 preparee pour affichage Web / envoi
String lastPreparedGPS103 = "";
unsigned long lastPreparedGPS103At = 0;

// Forward declarations utilisées par le gestionnaire SMS
void setGPSPower(bool state);
void setModemPower(bool state);
bool isValidIMEI(const String &s);
void taskModem();
bool saveConfig();
void readBattery();
String batteryText();
String batteryStatus();
void parseNetworkLine(const String &line);
void taskNetwork();
void ensureIMEI();
void taskTCP();
String buildGPS103Frame();
String buildGPS103Heartbeat();
void factoryReset();
void processGPS103ServerLogic(const String &line);
void processServerCommandPayload(const String &payload);
bool isServerCommandPayload(const String &payload);
String serverCommandFeedbackNumber();
String mqttCommandFeedbackNumber();
void taskServer();
void taskMovement();
void taskPower();
void taskWatchdog();
void enterModemSoftwareSleep();
void wakeModemSoftwareSleep();
void taskSendRecovery();
void taskLogger();
void startWiFi();
void stopWiFi();
void taskPendingSMSOTA();

// =========================
// ARCHITECTURE V5
// Tâches indépendantes : GPS / modem UART / réseau / TCP / store-forward /
// mouvement / énergie / Wi-Fi / Web. Aucune tâche réseau ne bloque la loop.
// Le modem est piloté par une machine d'état : BOOT -> AT -> SIM ->
// SIGNAL -> REGISTRATION -> ATTACH -> APN -> NETOPEN -> TCP.
// Toute perte de service revient automatiquement à l'étape concernée.
// =========================

// =========================
// A7670E / SMS (UART matériel Serial)
// Le modem est raccordé au Serial matériel ESP8266 :
//   A7670E TX -> GPIO3 (RX0)
//   A7670E RX -> GPIO1 (TX0)
// Le Serial USB ne doit donc pas être utilisé pour les logs
// lorsque le modem est alimenté. Les logs restent disponibles
// dans la page Web /logs.
// =========================
String modemLine = "";
String smsSender = "";
bool smsWaitingBody = false;

// Mise à jour distante demandée par SMS : la demande est planifiée hors du
// traitement de la ligne SMS afin de laisser la boucle principale piloter
// correctement l'initialisation du modem.
bool smsOTARequestPending = false;
String smsOTARequestURL = "";
String smsOTAReplyNumber = "";
unsigned long smsOTAStartNotBefore = 0;

unsigned long lastSMSInit = 0;
unsigned long lastSMSPoll = 0;
unsigned long modemLastActivity = 0;

// SMS non bloquant : aucune attente delay() pendant une transmission TCP.
bool smsTxBusy = false;
uint8_t smsTxStage = 0;              // 0 idle, 1 CMGS, 3 prompt, 4 confirmation
String smsTxNumber = "";
String smsTxMessage = "";
unsigned long smsTxStartedAt = 0;

// File d'attente SMS : une reponse ne doit jamais etre perdue parce qu'une
// autre transmission SMS/TCP est en cours. Les reponses restent courtes.
const uint8_t SMS_QUEUE_SIZE = 4;
String smsQueueNumber[SMS_QUEUE_SIZE];
String smsQueueMessage[SMS_QUEUE_SIZE];
uint8_t smsQueueHead = 0;
uint8_t smsQueueTail = 0;
uint8_t smsQueueCount = 0;

bool smsModemConfigured = false;
// Commande distante : coupe le modem seulement après l'envoi de l'accusé SMS.
bool remoteModemOffPending = false;

uint8_t smsConfigStep = 0;            // 0=CMGF, 1=CNMI, 2=CSCS, 3=termine

// =========================
// MODEM STATE MACHINE V5
// Une seule commande de supervision à la fois. Aucun delay bloquant.
// =========================
enum ModemInitStage : uint8_t {
  MODEM_STAGE_BOOT = 0,
  MODEM_STAGE_SYNC,
  MODEM_STAGE_SIM,
  MODEM_STAGE_SIGNAL,
  MODEM_STAGE_REGISTRATION,
  MODEM_STAGE_ATTACH,
  MODEM_STAGE_DATA,
  MODEM_STAGE_READY,
  MODEM_STAGE_RECOVERY
};
ModemInitStage modemStage = MODEM_STAGE_BOOT;
unsigned long modemStageSince = 0;
unsigned long modemLastCommandAt = 0;
unsigned long modemLastCommandTimeout = 0;
uint8_t modemFailureCount = 0;
uint8_t modemRecoveryCount = 0;
uint8_t netOpenFailureCount = 0;
const uint8_t NETOPEN_MAX_FAILURES = 5;
uint8_t modemBootCount = 0;
bool modemATReady = false;
bool modemDataReady = false;
bool modemWaitingResponse = false;
bool modemBasicConfigSent = false;
bool modemAPNConfigured = false;
bool modemTCPRxConfigured = false;
bool modemIMEIRequested = false;
String modemLastCommand = "";
void taskSMS();
void smsSend(const String &number, const String &message);
void taskTestModemRestart();

void clearSMSQueue() {
  for (uint8_t i = 0; i < SMS_QUEUE_SIZE; i++) {
    smsQueueNumber[i] = "";
    smsQueueMessage[i] = "";
  }
  smsQueueHead = 0;
  smsQueueTail = 0;
  smsQueueCount = 0;
}

const unsigned long MODEM_BOOT_WAIT_MS = 8000UL;
const unsigned long MODEM_CMD_GAP_MS = 300UL;
const unsigned long MODEM_CMD_TIMEOUT_MS = 5000UL;
const unsigned long MODEM_RECOVERY_DELAY_MS = 5000UL;

// Recuperation SIM hot-swap par coupure réelle de l'alimentation GPIO12.
// LOW = alimentation ON ; HIGH = alimentation OFF.
const unsigned long SIM_ABSENCE_CONFIRM_MS = 8000UL;
const unsigned long SIM_POWER_OFF_MS = 3000UL;
const unsigned long SIM_POWER_BOOT_WAIT_MS = 15000UL;
const unsigned long SIM_RESET_RETRY_MS = 60000UL;
bool simResetInProgress = false;
uint8_t simRecoveryPhase = 0; // 1 = alimentation coupée, 2 = attente démarrage
unsigned long simAbsentSince = 0;
unsigned long simResetStartedAt = 0;
unsigned long simResetLastAttemptAt = 0;

// TEST uniquement : redémarrage périodique du modem pour relancer
// l'initialisation et vérifier notamment la réinsertion de la SIM.
// Le reste du fonctionnement TEST reste continu et inchangé.
const unsigned long TEST_MODEM_RESTART_INTERVAL_MS = 100000UL; // 5 min
const unsigned long TEST_MODEM_RESTART_OFF_MS = 5000UL;        // 2 s
bool testModemRestarting = false;
unsigned long testModemRestartStarted = 0;
unsigned long testModemLastRestart = 0;

const uint8_t MODEM_MAX_SOFT_FAILURES = 5;
const uint8_t MODEM_MAX_POWER_CYCLES = 3;
unsigned long modemRecoverySince = 0;
void modemSendAT(const String &cmd);

void taskSMS() {
  if (!modemPower) return;

  // Configuration SMS : réception directe +CMT.
  if (!smsModemConfigured && modemStage >= MODEM_STAGE_SIM &&
      !modemTcpBusy && !modemExpectSendPrompt && !smsTxBusy &&
      !modemWaitingResponse && millis() - modemLastCommandAt >= MODEM_CMD_GAP_MS) {
    if (smsConfigStep == 0) {
      smsConfigStep = 1;
      modemSendAT("AT+CMGF=1");
    } else if (smsConfigStep == 1) {
      smsConfigStep = 2;
      modemSendAT("AT+CNMI=2,2,0,0,0");
    } else if (smsConfigStep == 2) {
      smsConfigStep = 3;
      modemSendAT("AT+CSCS=\"GSM\"");
    } else {
      smsModemConfigured = true;
      addLog("SMS RX READY (+CMT)");
    }
  }

  // Une seule transmission a la fois, mais les reponses sont mises en file.
  // Cela evite le probleme "SMS TX BUSY -> response skipped".
  if (!smsTxBusy && smsQueueCount > 0 && !modemTcpBusy && !modemExpectSendPrompt && !modemWaitingResponse) {
    smsTxNumber = smsQueueNumber[smsQueueHead];
    smsTxMessage = smsQueueMessage[smsQueueHead];
    smsQueueNumber[smsQueueHead] = "";
    smsQueueMessage[smsQueueHead] = "";
    smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
    smsQueueCount--;
    smsTxStartedAt = millis();
    smsTxStage = 1;
    smsTxBusy = true;
  }

  if (!smsTxBusy) return;
  if (millis() - smsTxStartedAt > 20000UL) {
    smsTxBusy = false; smsTxStage = 0; smsTxNumber = ""; smsTxMessage = "";
    addLog("SMS TX TIMEOUT");
    return;
  }

  if (modemTcpBusy || modemExpectSendPrompt) return;

  if (smsTxStage == 1 && !modemWaitingResponse) {
    // SMS en mode GSM standard : pas d'UCS2 et pas de caracteres emoji.
    Serial.print("AT+CMGS=\"");
    Serial.print(smsTxNumber);
    Serial.print("\"");
    Serial.print("\r");
    modemLastActivity = millis();
    modemLastCommand = "AT+CMGS";
    modemWaitingResponse = true;
    smsTxStage = 3;
  }
}

// Réception TCP en mode +RECEIVE,<socket>,<length>.
uint16_t modemRxExpected = 0;
String modemRxPayload = "";

void modemSendAT(const String &cmd) {
  if (!modemPower) return;
  traceModemTX(cmd);
  Serial.print(cmd);
  Serial.print("\r");
  modemLastActivity = millis();
  modemLastCommand = cmd;
  modemWaitingResponse = true;
  modemLastCommandTimeout = millis() + MODEM_CMD_TIMEOUT_MS;
}

String normalizePhone(String n) {
  n.trim();
  n.replace(" ", "");
  n.replace("-", "");
  if (n.startsWith("00")) n = "+" + n.substring(2);
  return n;
}

bool smsAuthorized(const String &number) {
  String n = normalizePhone(number);
  if (!n.length()) return false;
  String a1=normalizePhone(cfg.smsAdmin1), a2=normalizePhone(cfg.smsAdmin2), a3=normalizePhone(cfg.smsAdmin3);
  if (a1.length() && n==a1) return true;
  if (a2.length() && n==a2) return true;
  if (a3.length() && n==a3) return true;
  // Tolérance format local marocain 06/07 <-> +2126/7.
  if (n.startsWith("+212") && n.length()==13) {
    String local="0"+n.substring(4);
    if ((a1.length() && local==a1)||(a2.length() && local==a2)||(a3.length() && local==a3)) return true;
  }
  return false;
}

bool validPhoneNumber(const String &number) {
  String n = normalizePhone(number);
  if (n.length() < 8 || n.length() > 16) return false;
  uint8_t start = n.startsWith("+") ? 1 : 0;
  if (start && n.length() < 9) return false;
  for (uint8_t i = start; i < n.length(); i++) if (!isDigit(n[i])) return false;
  return true;
}

String cleanSMS(String s) {
  s.trim();
  s.toUpperCase();
  return s;
}

void smsSend(const String &number, const String &message) {
  if (!modemPower || number.length() == 0 || message.length() == 0) return;

  // Limite volontaire : SMS de service court et fiable.
  String msg = message;
  if (msg.length() > 150) {
    msg.remove(147);
    msg += "...";
  }

  // Toujours mettre en file : aucune reponse ne doit etre perdue si un SMS
  // precedent ou une communication est encore en cours.
  if (smsQueueCount >= SMS_QUEUE_SIZE) {
    // En cas de saturation, conserver les reponses les plus recentes.
    smsQueueNumber[smsQueueHead] = "";
    smsQueueMessage[smsQueueHead] = "";
    smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
    smsQueueCount--;
  }

  smsQueueNumber[smsQueueTail] = number;
  smsQueueMessage[smsQueueTail] = msg;
  smsQueueTail = (smsQueueTail + 1) % SMS_QUEUE_SIZE;
  smsQueueCount++;

  // Pendant une commande distante, on capture la meme reponse que celle
  // produite pour le SMS. Elle sera publiee apres la fin de handleSMSCommand(),
  // jamais depuis smsSend(), afin d'eviter une commande imbriquee pendant
  // une transaction de communication.
  if (mqttCommandContext) {
    mqttCommandResponse = msg;
  }

  addLog("SMS TX QUEUED");
}

void smsTxPrompt() {
  if (!smsTxBusy || smsTxStage != 3) return;
  // Message ASCII/GSM deja normalise.
  traceModemTX("[SMS] " + smsTxMessage);
  Serial.print(smsTxMessage);
  Serial.write(26);
  Serial.flush();
  modemLastActivity = millis();
  smsTxStage = 4;
  modemWaitingResponse = true;
  addLog("SMS TX SEND -> " + smsTxNumber);
}

void smsTxComplete(bool ok) {
  if (!smsTxBusy) return;
  addLog(ok ? "SMS TX OK" : "SMS TX ERROR");
  if (pendingLocationSending) {
    // Pour une demande provenant du canal de commande, sa suppression est
    // gérée après confirmation sur ce meme canal.
    if (ok && !pendingLocationViaMQTT) clearPendingLocationRequest();
    pendingLocationSending = false;
  }
  smsTxBusy = false;
  smsTxStage = 0;
  smsTxNumber = "";
  smsTxMessage = "";
  modemWaitingResponse = false;
  if (remoteModemOffPending) {
    remoteModemOffPending = false;
    if (ok) {
      delay(100);
      setModemPower(false);
    }
  }
}

String smsNormalize(const String &input) {
  String s = input;
  // Normaliser les lettres accentuees courantes avant envoi SMS.
  const char* from[] = {"à","á","â","ä","ã","å","À","Á","Â","Ä","Ã","Å",
                        "ç","Ç","è","é","ê","ë","È","É","Ê","Ë",
                        "ì","í","î","ï","Ì","Í","Î","Ï",
                        "ò","ó","ô","ö","õ","Ò","Ó","Ô","Ö","Õ",
                        "ù","ú","û","ü","Ù","Ú","Û","Ü","ÿ","Ÿ",
                        "œ","Œ","æ","Æ"};
  const char* to[]   = {"a","a","a","a","a","a","A","A","A","A","A","A",
                        "c","C","e","e","e","e","E","E","E","E",
                        "i","i","i","i","I","I","I","I",
                        "o","o","o","o","o","O","O","O","O","O",
                        "u","u","u","u","U","U","U","U","y","Y",
                        "oe","OE","ae","AE"};
  for (uint8_t i = 0; i < sizeof(from) / sizeof(from[0]); i++) s.replace(from[i], to[i]);

  // Conserver uniquement les caracteres utiles a un SMS de service.
  // Seuls les caracteres ASCII/GSM utiles sont conserves.
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 128 && (isAlphaNumeric(c) || c == ' ' || c == '\n' || c == '\r' ||
        c == ':' || c == '/' || c == '.' || c == '_' || c == '-' || c == '=' ||
        c == '%' || c == '?' || c == ',' || c == '+')) {
      out += (char)c;
    }
  }
  out.trim();
  return out;
}

void smsReply(const String &message) {
  String msg = smsNormalize(message);

  // Une commande MQTT doit recevoir sa réponse même sans numéro SMS.
  if (mqttCommandContext) {
    mqttCommandResponse = msg;
    if (smsSender.length()) smsSend(smsSender, msg);
    return;
  }

  if (smsSender.length()) smsSend(smsSender, msg);
}

void smsNoReply() {
  // Sécurité : aucun SMS n'est envoyé si l'authentification échoue.
}

bool verifySMSPassword(const String &provided) {
  return provided.length() > 0 && provided == cfg.smsPassword;
}

bool verifySMSPrivateCode(const String &provided) {
  return provided.length() > 0 && cfg.smsPrivateCode.length() >= 4 && provided == cfg.smsPrivateCode;
}

bool verifySMSAccess(const String &provided) {
  return verifySMSPassword(provided) || verifySMSPrivateCode(provided);
}

bool splitLastToken(const String &input, String &body, String &password) {
  String t = input;
  t.trim();
  int p = t.lastIndexOf('_');
  if (p <= 0 || p >= (int)t.length() - 1) return false;
  body = t.substring(0, p);
  body.trim();
  body.replace('_', ' ');
  password = t.substring(p + 1);
  password.trim();
  return body.length() > 0 && password.length() > 0;
}

String locationSMS() {
  if (!gps.fix) return "LOCATION\nGPS : NO FIX";
  String r = "LOCATION\n";
  r += "DATE : " + gps.date + " " + gps.utc + " UTC\n";
  r += "VITESSE : " + String(gps.speed, 1) + " km/h\n";
  r += "LIEN MAP : https://maps.google.com/?q=" + String(gps.lat, 6) + "," + String(gps.lon, 6);
  return r;
}


// ============================================================
// OTA PAR URL VIA MODEM 4G
// Le firmware est telecharge par HTTP(S) depuis l URL fournie.
// Le noyau TCP GPS103 reste independant de cette fonction.
// ============================================================
// Pendant une mise a jour 4G, la procedure OTA peut durer longtemps.
// Cette petite pompe laisse le serveur Web repondre aux requetes de la
// page OTA (/api/trace et /api/ota-status) sans modifier la logique OTA.
void otaWebPump();

void otaFlushModemInput() {
  while (Serial.available()) Serial.read();
}

void otaWebPump() {
  // Ne traiter les requêtes HTTP que lorsque le point d'accès Wi-Fi est actif.
  if (wifiEnabled) server.handleClient();
  yield();
}

bool otaWaitText(const String &needle, uint32_t timeoutMs, String *responseOut = nullptr) {
  String r;
  r.reserve(256);
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      r += c;
      if (r.length() > 400) r.remove(0, r.length() - 400);
      if (r.indexOf(needle) >= 0) {
        if (responseOut) *responseOut = r;
        return true;
      }
      if (r.indexOf("ERROR") >= 0) {
        if (responseOut) *responseOut = r;
        return false;
      }
    }
    otaWebPump();
  }
  if (responseOut) *responseOut = r;
  return false;
}

bool otaAT(const String &cmd, const String &okText, uint32_t timeoutMs) {
  otaFlushModemInput();
  traceOTATX(cmd);
  if (otaWebStreamActive) {
    server.sendContent("AT> " + cmd + "\n");
  }
  Serial.println(cmd);

  String response;
  bool ok = otaWaitText(okText, timeoutMs, &response);

  if (otaWebStreamActive) {
    response.trim();
    if (response.length()) {
      traceOTARX(response);
      server.sendContent("AT< " + response + "\n");
    } else {
      traceOTARX("[TIMEOUT]");
      server.sendContent("AT< [TIMEOUT]\n");
    }
  }
  return ok;
}

bool otaGetAction(uint32_t &dataLen, int &statusCode) {
  String r;
  r.reserve(180);
  uint32_t start = millis();
  while (millis() - start < 125000UL) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      r += c;
      if (r.length() > 500) r.remove(0, r.length() - 500);
      int p = r.lastIndexOf("+HTTPACTION:");
      if (p >= 0) {
        String line = r.substring(p);
        int a = line.indexOf(',');
        int b = line.indexOf(',', a + 1);
        if (a > 0 && b > a) {
          statusCode = line.substring(a + 1, b).toInt();
          int e = line.indexOf('\n', b);
          String n = (e > b) ? line.substring(b + 1, e) : line.substring(b + 1);
          dataLen = (uint32_t)n.toInt();
          if (otaWebStreamActive) {
            String action = line.substring(0, line.indexOf('\n') >= 0 ? line.indexOf('\n') : line.length());
            action.trim();
            traceOTARX(action);
            server.sendContent("AT< " + action + "\n");
          }
          return true;
        }
      }
    }
    otaWebPump();
  }
  return false;
}

bool otaReadHTTPChunk(File &dest, uint32_t offset, size_t wanted, size_t &received) {
  received = 0;
  otaFlushModemInput();

  traceOTATX("AT+HTTPREAD=" + String(offset) + "," + String(wanted));
  if (otaWebStreamActive) {
    server.sendContent("AT> AT+HTTPREAD=" + String(offset) + "," + String(wanted) + "\n");
  }
  Serial.print("AT+HTTPREAD=");
  Serial.print(offset);
  Serial.print(",");
  Serial.println(wanted);

  String head;
  head.reserve(100);
  uint32_t start = millis();
  int n = -1;

  while (millis() - start < 120000UL) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      head += c;
      if (head.length() > 160) head.remove(0, head.length() - 160);

      int p = head.lastIndexOf("+HTTPREAD:");
      if (p >= 0) {
        int e = head.indexOf('\n', p);
        if (e >= 0) {
          String v = head.substring(p + 10, e);
          v.trim();
          n = v.toInt();
          if (otaWebStreamActive) server.sendContent("AT< +HTTPREAD: " + v + "\n");
          break;
        }
      }
      if (head.indexOf("ERROR") >= 0) return false;
    }
    if (n >= 0) break;
    otaWebPump();
  }

  // 0 = fin réelle du contenu disponible.
  if (n == 0) {
    received = 0;
    return true;
  }

  if (n < 0 || (size_t)n > wanted) return false;

  uint8_t buf[MODEM_OTA_CHUNK];
  size_t left = (size_t)n;
  const uint32_t readDeadline = millis() + 120000UL;

  while (left && (int32_t)(readDeadline - millis()) > 0) {
    size_t avail = Serial.available();
    if (!avail) { yield(); continue; }

    size_t ask = left;
    if (ask > sizeof(buf)) ask = sizeof(buf);
    if (avail < ask) ask = avail;

    size_t got = Serial.readBytes((char*)buf, ask);
    if (!got) { yield(); continue; }

    if (dest.write(buf, got) != got) return false;
    received += got;
    left -= got;
    otaWebPump();
  }

  if (left != 0 || received != (size_t)n) return false;

  // La réponse +HTTPREAD:0 termine chaque lecture dans ce modem.
  String endResp;
  bool endOK = otaWaitText("+HTTPREAD: 0", 15000, &endResp);
  if (otaWebStreamActive) {
    endResp.trim();
    if (endResp.length()) server.sendContent("AT< " + endResp + "\n");
    else server.sendContent("AT< [TIMEOUT]\n");
  }
  return endOK;
}

// Vérification indépendante du fichier ESP8266 avant toute programmation.
// Vérifie le magic, les segments, la taille réelle et le checksum final.
bool validateESP8266FirmwareFile(File &f, String &reason) {
  reason = "";
  const size_t fileSize = f.size();
  if (fileSize < 64) { reason = "FICHIER TROP PETIT"; return false; }
  if (!f.seek(0, SeekSet)) { reason = "LECTURE FICHIER"; return false; }

  // Les BIN produits par l'outil Arduino ESP8266 peuvent contenir un
  // bootloader occupe jusqu'a 4096 octets puis l'image application.
  // Le fichier de reference fourni montre exactement cette organisation.
  uint8_t h0[8];
  if (f.read(h0, sizeof(h0)) != sizeof(h0)) { reason = "EN-TETE INCOMPLETE"; return false; }
  if (h0[0] != 0xE9) { reason = "MAGIC ESP8266 INVALIDE"; return false; }

  // Recherche de la seconde image a l'offset 0x1000. Les images Arduino
  // combinees utilisent ce point pour l'application.
  const size_t appOffset = 0x1000;
  if (fileSize <= appOffset + 8) { reason = "IMAGE APPLICATION ABSENTE"; return false; }

  if (!f.seek(appOffset, SeekSet)) { reason = "LECTURE IMAGE APPLICATION"; return false; }
  uint8_t ha[8];
  if (f.read(ha, sizeof(ha)) != sizeof(ha)) { reason = "EN-TETE APPLICATION INCOMPLETE"; return false; }
  if (ha[0] != 0xE9) { reason = "MAGIC APPLICATION INVALIDE"; return false; }

  uint8_t segments = ha[1];
  if (segments == 0 || segments > 16) { reason = "SEGMENTS APPLICATION INVALIDES"; return false; }

  // Parse de tous les segments de l'application. On ne suppose aucune
  // taille de firmware particuliere.
  uint8_t buf[256];
  uint32_t dataBytes = 0;
  for (uint8_t seg = 0; seg < segments; seg++) {
    uint8_t sh[8];
    if (f.read(sh, sizeof(sh)) != sizeof(sh)) { reason = "SEGMENT APPLICATION INCOMPLET"; return false; }

    uint32_t addr = (uint32_t)sh[0] |
                    ((uint32_t)sh[1] << 8) |
                    ((uint32_t)sh[2] << 16) |
                    ((uint32_t)sh[3] << 24);
    uint32_t len = (uint32_t)sh[4] |
                   ((uint32_t)sh[5] << 8) |
                   ((uint32_t)sh[6] << 16) |
                   ((uint32_t)sh[7] << 24);
    (void)addr;

    if (len == 0) { reason = "SEGMENT VIDE"; return false; }
    if ((uint64_t)dataBytes + len > (uint64_t)fileSize) { reason = "TAILLE SEGMENT INVALIDE"; return false; }

    uint32_t left = len;
    while (left) {
      size_t n = left > sizeof(buf) ? sizeof(buf) : left;
      if (f.read(buf, n) != n) { reason = "DONNEES SEGMENT INCOMPLETES"; return false; }
      left -= (uint32_t)n;
    }
    dataBytes += len;
    otaWebPump();
  }

  // La fin de l'image application est alignee sur 16 octets : des zeros
  // peuvent preceder le dernier octet checksum. On accepte ce padding sans
  // imposer une longueur particuliere.
  size_t footerStart = (size_t)f.position();
  if (footerStart >= fileSize) { reason = "FOOTER ABSENT"; return false; }

  size_t checksumPos = fileSize - 1;
  if (checksumPos < footerStart) { reason = "TAILLE IMAGE INVALIDE"; return false; }

  // Si le fichier contient le CRC Arduino global, c'est cette verification
  // qui est prioritaire. Le CRC est stocke a 0x1010 et 0x1014 et couvre tout
  // le fichier apres remise a zero de ces deux champs.
  bool crcPresent = false;
  bool crcValid = false;
  if (fileSize >= 0x1018) {
    uint8_t meta[8];
    if (f.seek(0x1010, SeekSet) && f.read(meta, sizeof(meta)) == sizeof(meta)) {
      uint32_t crcSize = (uint32_t)meta[0] |
                         ((uint32_t)meta[1] << 8) |
                         ((uint32_t)meta[2] << 16) |
                         ((uint32_t)meta[3] << 24);
      uint32_t storedCRC = (uint32_t)meta[4] |
                           ((uint32_t)meta[5] << 8) |
                           ((uint32_t)meta[6] << 16) |
                           ((uint32_t)meta[7] << 24);
      if (crcSize == fileSize && storedCRC != 0 && storedCRC != 0xFFFFFFFFUL) {
        crcPresent = true;

        uint32_t crc = 0xFFFFFFFFUL;
        if (!f.seek(0, SeekSet)) { reason = "LECTURE CRC"; return false; }
        size_t pos = 0;
        while (pos < fileSize) {
          size_t n = fileSize - pos;
          if (n > sizeof(buf)) n = sizeof(buf);
          if (f.read(buf, n) != n) { reason = "LECTURE CRC INCOMPLETE"; return false; }

          for (size_t i = 0; i < n; i++) {
            size_t absolute = pos + i;
            uint8_t v = buf[i];
            if (absolute >= 0x1010 && absolute < 0x1018) v = 0;
            for (uint8_t bit = 0x80; bit; bit >>= 1) {
              bool x = (v & bit) != 0;
              bool top = (crc & 0x80000000UL) != 0;
              crc <<= 1;
              if (x != top) crc ^= 0x04C11DB7UL;
            }
          }
          pos += n;
          otaWebPump();
        }
        crcValid = (crc == storedCRC);
        if (!crcValid) { reason = "CRC IMAGE INVALIDE"; return false; }
      }
    }
  }

  // Pour les anciens BIN sans CRC global, verifier le checksum ESP8266
  // traditionnel. Pour les BIN Arduino avec CRC, le checksum peut avoir ete
  // rendu incoherent par la mise a jour CRC : le CRC global est alors la
  // verification d'integrite de reference.
  if (!crcPresent) {
    uint8_t checksum = 0xEF;
    if (!f.seek(appOffset, SeekSet)) { reason = "LECTURE CHECKSUM"; return false; }
    uint8_t ih[8];
    if (f.read(ih, sizeof(ih)) != sizeof(ih)) { reason = "EN-TETE CHECKSUM"; return false; }
    uint8_t nseg = ih[1];
    for (uint8_t seg = 0; seg < nseg; seg++) {
      uint8_t sh[8];
      if (f.read(sh, sizeof(sh)) != sizeof(sh)) { reason = "CHECKSUM SEGMENT"; return false; }
      uint32_t len = (uint32_t)sh[4] |
                     ((uint32_t)sh[5] << 8) |
                     ((uint32_t)sh[6] << 16) |
                     ((uint32_t)sh[7] << 24);
      uint32_t left = len;
      while (left) {
        size_t n = left > sizeof(buf) ? sizeof(buf) : left;
        if (f.read(buf, n) != n) { reason = "CHECKSUM DONNEES"; return false; }
        for (size_t i = 0; i < n; i++) checksum ^= buf[i];
        left -= (uint32_t)n;
      }
    }

    if (!f.seek(checksumPos, SeekSet)) { reason = "CHECKSUM FINAL"; return false; }
    int finalByte = f.read();
    if (finalByte < 0) { reason = "CHECKSUM ABSENT"; return false; }
    if ((uint8_t)finalByte != checksum) { reason = "CHECKSUM INVALIDE"; return false; }
  }

  if (!crcValid && crcPresent) { reason = "CRC IMAGE INVALIDE"; return false; }

  if (!f.seek(0, SeekSet)) { reason = "VERIFICATION FINALE"; return false; }
  if (fileSize > ESP.getFreeSketchSpace()) { reason = "FIRMWARE TROP GRAND"; return false; }

  return true;
}

bool installValidatedFirmwareFile(File &f) {
  const size_t size = f.size();
  if (size == 0 || size > ESP.getFreeSketchSpace()) return false;
  if (!f.seek(0, SeekSet)) return false;

  if (!Update.begin(size, U_FLASH)) return false;

  uint8_t buf[1024];
  size_t total = 0;
  while (total < size) {
    size_t want = size - total;
    if (want > sizeof(buf)) want = sizeof(buf);
    size_t got = f.read(buf, want);
    if (got != want) { Update.end(false); return false; }
    if (Update.write(buf, got) != got) { Update.end(false); return false; }
    total += got;
    if (otaWebStreamActive && size) {
      uint8_t p = (uint8_t)((total * 100UL) / size);
      if (p != otaProgressPercent) {
        otaProgressPercent = p;
        otaWebLog("Programmation : " + String(p) + "%");
      }
    }
    otaWebPump();
  }

  if (!Update.end(true) || !Update.isFinished()) return false;
  return true;
}

bool prepareAndInstallTempFirmware(const char* label) {
  File f = SPIFFS.open(OTA_TEMP_FILE, "r");
  if (!f) { otaLastError = "FICHIER TEMPORAIRE"; otaWebLog("Fichier temporaire absent"); return false; }

  String reason;
  otaWebLog(String("Vérification : ") + label);
  bool valid = validateESP8266FirmwareFile(f, reason);
  if (!valid) {
    f.close();
    SPIFFS.remove(OTA_TEMP_FILE);
    otaLastError = "FIRMWARE INVALIDE";
    otaWebLog("Firmware refusé : " + reason);
    return false;
  }

  otaWebLog("Firmware valide : " + String(f.size()) + " octets");
  otaWebLog("Programmation de la mémoire...");
  otaProgressPercent = 0;
  bool ok = installValidatedFirmwareFile(f);
  f.close();
  SPIFFS.remove(OTA_TEMP_FILE);

  if (!ok) {
    otaLastError = "PROGRAMMATION";
    otaWebLog("Programmation échouée");
    return false;
  }

  otaProgressPercent = 100;
  otaWebLog("Firmware installé et vérifié");
  return true;
}

void handleLocalFirmwareUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUploadFailed = false;
    otaUploadReceived = 0;
    otaLocalDirectMode = true;
    otaLocalUpdateStarted = false;
    otaOTAInProgress = false;
    otaLocalUploadValidator = OTAStreamValidator();

    // Ne pas stocker le .bin complet dans SPIFFS : le fichier de 256 Ko
    // peut dépasser l'espace libre. Programmer au fil de la réception.
    cleanupTemporaryFiles();
    SPIFFS.remove(OTA_TEMP_FILE);
    otaLastError = "";
    otaBytesReceived = 0;
    otaTotalBytes = upload.totalSize;
    otaProgressPercent = 0;
    otaSetProgress(0, upload.totalSize, "Préparation de la réception Wi-Fi...");

    // Au début d'un multipart HTTP, totalSize peut encore être nul ou
    // incomplet. Réserver l'espace OTA disponible et déterminer la taille
    // réelle à partir des octets reçus, puis valider à la fin.
    if (!otaBeginCommon(0, otaLocalUploadValidator)) {
      otaUploadFailed = true;
    } else {
      otaLocalUpdateStarted = true;
      otaWebLog("Réception directe : aucun fichier temporaire SPIFFS");
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (otaUploadFailed || !otaLocalUpdateStarted) return;
    if (!otaWriteCommon(upload.buf, upload.currentSize, otaLocalUploadValidator)) {
      otaUploadFailed = true;
      Update.end(false);
      otaLocalUpdateStarted = false;
      return;
    }
    otaUploadReceived += upload.currentSize;
    // IMPORTANT : ne pas rappeler server.handleClient() depuis le callback
    // d'upload multipart. Cela peut réentrer dans le serveur HTTP et couper
    // le transfert du navigateur. Laisser seulement l'ESP8266 respirer.
    yield();
  } else if (upload.status == UPLOAD_FILE_END) {
    // La validation finale est faite dans le gestionnaire HTTP_POST,
    // après réception complète du multipart.
    if (!otaLocalUpdateStarted) otaUploadFailed = true;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (otaLocalUpdateStarted) Update.end(false);
    otaLocalUpdateStarted = false;
    otaUploadFailed = true;
    otaLastError = "TRANSFERT INTERROMPU";
    otaLocalDirectMode = false;
  }
}


String buildGitHubFirmwareURL(String fileName) {
  fileName.trim();
  if (!fileName.length()) return String();

  // L'utilisateur fournit uniquement le nom/version du firmware.
  // Exemples : AI-3.ino  -> .../AI-3.ino.bin
  //            AI-3      -> .../AI-3.bin
  String lower = fileName;
  lower.toLowerCase();
  if (lower.endsWith(".bin")) {
    fileName.remove(fileName.length() - 4);
    lower.remove(lower.length() - 4);
  }

  // Sécurité : ne jamais accepter une URL dans le champ version.
  if (fileName.indexOf("://") >= 0 || fileName.indexOf('/') >= 0 || fileName.indexOf('\\') >= 0)
    return String();
  if (fileName.indexOf(' ') >= 0 || fileName.indexOf('\r') >= 0 || fileName.indexOf('\n') >= 0)
    return String();
  if (fileName.length() < 1 || fileName.length() > 80) return String();

  return String(OTA_GITHUB_FIRMWARE_BASE) + fileName + ".bin";
}

String normalizeFirmwareURL(String url) {
  url.trim();

  // GitHub "blob" pages are HTML, not firmware binaries.
  // Convert automatically to the raw file URL.
  if (url.startsWith("https://github.com/") || url.startsWith("http://github.com/")) {
    int base = url.indexOf("://github.com/");
    if (base >= 0) {
      int start = base + 14; // points after ://github.com/
      int blob = url.indexOf("/blob/", start);
      if (blob > start) {
        String repo = url.substring(start, blob);
        String tail = url.substring(blob + 6);
        int slash = tail.indexOf('/');
        if (slash > 0 && slash < (int)tail.length() - 1) {
          String branch = tail.substring(0, slash);
          String filePath = tail.substring(slash + 1);

          // Remove optional query string such as ?raw=1
          int q = filePath.indexOf('?');
          if (q >= 0) filePath = filePath.substring(0, q);

          return "https://raw.githubusercontent.com/" + repo + "/" + branch + "/" + filePath;
        }
      }
    }
  }

  return url;
}

void stopAllCommunicationsForOTA() {
  // Pendant un OTA : aucune lecture GPS, aucun SMS, aucune transmission
  // serveur et aucune tentative de reconnexion ne doivent utiliser le modem.
  setGPSPower(false);

  modemTcpBusy = false;
  modemExpectSendPrompt = false;
  modemWaitingResponse = false;
  smsTxBusy = false;
  smsTxStage = 0;
  smsTxNumber = "";
  smsTxMessage = "";
  clearSMSQueue();
  modemRxExpected = 0;
  modemRxPayload = "";

  link1.connected = false;
  link1.opening = false;
  link1.sending = false;
  link1.loginSent = false;
  link2.connected = false;
  link2.opening = false;
  link2.sending = false;
  link2.loginSent = false;

  tcpServiceOpen = false;
  modemDataReady = false;
  packetServiceAttached = false;

  otaFlushModemInput();

  // Fermer proprement les anciennes sessions IP avant HTTP(S).
  otaAT("AT+CIPCLOSE=0", "OK", 3000);
  otaAT("AT+CIPCLOSE=1", "OK", 3000);
  otaAT("AT+NETCLOSE", "OK", 10000);
  otaAT("AT+HTTPTERM", "OK", 3000);
  otaFlushModemInput();
}

bool ensureModemReadyForOTA(uint32_t timeoutMs = 35000UL) {
  const uint32_t start = millis();
  const bool wasAlreadyPowered = modemPower;
  uint32_t minimumPowerWaitUntil = start;
  otaWebLog("Vérification de la communication avant téléchargement...");

  if (!wasAlreadyPowered) {
    otaWebLog("Communication inactive -> activation...");
    setModemPower(true);
    // Après une mise sous tension, laisser au modem au minimum 30 s
    // avant d'autoriser le démarrage du téléchargement.
    minimumPowerWaitUntil = start + 30000UL;
    otaWebLog("Stabilisation : 30 secondes minimum...");
  } else {
    otaWebLog("Communication déjà active -> vérification de la disponibilité...");
  }

  uint8_t lastStage = 255;
  while (millis() - start < timeoutMs) {
    if (!modemPower) {
      otaWebLog("Alimentation modem perdue -> nouvelle mise sous tension...");
      setModemPower(true);
      minimumPowerWaitUntil = millis() + 15000UL;
      otaWebLog("Nouvelle stabilisation : 15 secondes minimum...");
    }

    // La machine d'état normale initialise le modem sans lancer l'OTA.
    // Pour l'OTA, une session TCP existante n'est PAS obligatoire :
    // le téléchargement utilisera ensuite NETOPEN/HTTP.
    taskModem();
    taskNetwork();

    // Pour l'OTA, NETOPEN/HTTP sera établi juste après.
    // Il ne faut donc pas exiger ici les indicateurs internes
    // packetServiceAttached/modemDataReady : ils peuvent être faux
    // alors que le modem, la SIM et le réseau sont réellement prêts.
    // Cette vérification évite notamment le faux échec lorsque le
    // service IP est déjà ouvert dans le modem.
    // La machine d'état peut déjà être en READY alors que certains
    // indicateurs internes (modemATReady/simReady/networkRegistered)
    // ne sont pas encore synchronisés. Pour l'OTA, l'état READY est
    // suffisant : NETOPEN/HTTP sera vérifié ensuite directement par AT.
    if (millis() >= minimumPowerWaitUntil &&
        (modemStage == MODEM_STAGE_READY ||
         (modemATReady && simReady && networkRegistered))) {
      otaWebLog("Communication prête : modem + SIM + réseau disponibles");
      return true;
    }

    uint8_t stage = (uint8_t)modemStage;
    if (stage != lastStage) {
      lastStage = stage;
      switch (modemStage) {
        case MODEM_STAGE_BOOT:         otaWebLog("Communication : démarrage..."); break;
        case MODEM_STAGE_SYNC:         otaWebLog("Communication : synchronisation..."); break;
        case MODEM_STAGE_SIM:          otaWebLog("Communication : vérification de l'accès..."); break;
        case MODEM_STAGE_SIGNAL:       otaWebLog("Communication : recherche du réseau..."); break;
        case MODEM_STAGE_REGISTRATION: otaWebLog("Communication : enregistrement..."); break;
        case MODEM_STAGE_ATTACH:       otaWebLog("Communication : accès Internet..."); break;
        case MODEM_STAGE_DATA:         otaWebLog("Communication : ouverture du service..."); break;
        case MODEM_STAGE_READY:        otaWebLog("Communication : prête..."); break;
        case MODEM_STAGE_RECOVERY:     otaWebLog("Communication : récupération..."); break;
      }
    }
    otaWebPump();
    delay(5);
  }

  otaLastError = "MODEM NON PRET";
  otaWebLog("La communication n'est pas prête : téléchargement annulé");
  return false;
}


// ============================================================
// OTA 4G DIRECTE : téléchargement directement vers la partition OTA
// Aucun fichier firmware complet n'est créé dans SPIFFS.
// Les fichiers/configurations et le buffer GPS restent intacts.
// ============================================================


// ============================================================
// OTA COMMUN : validation + ecriture directe dans la partition OTA
// La source des segments peut etre le WiFi ou le modem 4G.
// Aucun firmware complet n'est stocke dans SPIFFS.
// ============================================================
void otaSetProgress(uint32_t received, uint32_t total, const String &stage) {
  otaBytesReceived = received;
  otaTotalBytes = total;
  otaStage = stage;
  uint8_t p = 0;
  if (total > 0) {
    uint32_t q = (received * 100UL) / total;
    if (q > 100) q = 100;
    p = (uint8_t)q;
  }
  otaProgressPercent = p;
  otaStatusMessage = stage;
}

bool otaBeginCommon(uint32_t total, OTAStreamValidator &validator) {
  // Content-Length n'est pas obligatoire : le moteur peut programmer
  // directement un flux de taille inconnue et laisser le validateur décider
  // à la fin si l'image est complète et valide.
  validator.total = total;
  validator.totalKnown = (total > 0);

  if (validator.totalKnown && (size_t)total > ESP.getFreeSketchSpace()) {
    otaLastError = "FIRMWARE TROP GRAND";
    otaSetProgress(0, total, "Firmware trop grand");
    return false;
  }

  // Lorsque Content-Length est inconnu, ne pas passer 0 à Update.begin().
  // Certaines versions de la bibliotheque ESP8266 Update gerent mal ce cas.
  // On reserve alors explicitement tout l'espace sketch disponible, aligne
  // sur une taille de secteur. Le validateur controlera la taille reelle
  // recue a la fin avant d'autoriser l'installation.
  size_t freeSpace = ESP.getFreeSketchSpace();
  size_t updateSize = validator.totalKnown ? (size_t)total
                                          : (freeSpace > 0x1000UL ? ((freeSpace - 0x1000UL) & 0xFFFFF000UL) : 0);

  otaWebLog("Espace OTA disponible : " + String((uint32_t)freeSpace) + " octets");
  otaWebLog("Espace reserve pour OTA : " + String((uint32_t)updateSize) + " octets");

  if (updateSize == 0 || updateSize > freeSpace || !Update.begin(updateSize, U_FLASH)) {
    otaLastError = "MEMOIRE OTA";
    otaSetProgress(0, total, "Impossible d'ouvrir la partition OTA");
    otaWebLog("Update.begin() refuse l'espace OTA disponible");
    return false;
  }
  otaOTAInProgress = true;
  otaSetProgress(0, total, "Programmation OTA : 0%");
  return true;
}

bool otaWriteCommon(const uint8_t *data, size_t len, OTAStreamValidator &validator) {
  if (!len) return true;
  if (!validator.feed(data, len)) {
    otaLastError = "FIRMWARE INVALIDE";
    otaSetProgress(otaBytesReceived, otaTotalBytes, "Firmware invalide : " + validator.reason);
    return false;
  }
  if (Update.write(const_cast<uint8_t*>(data), len) != len) {
    otaLastError = "ECRITURE OTA";
    otaSetProgress(otaBytesReceived, otaTotalBytes, "Erreur d'ecriture OTA");
    return false;
  }
  otaBytesReceived += (uint32_t)len;
  uint8_t p = 0;
  if (otaTotalBytes) {
    uint32_t q = (otaBytesReceived * 100UL) / otaTotalBytes;
    if (q > 100) q = 100;
    p = (uint8_t)q;
  }
  otaProgressPercent = p;
  otaStage = otaTotalBytes ? ("Téléchargement : " + String(p) + "%")
                           : ("Téléchargement : " + String(otaBytesReceived) + " octets");
  static uint32_t lastOtaLogBytes = 0;
  // Pendant l'upload local multipart, ne jamais écrire de texte dans la
  // réponse HTTP en cours. La progression reste disponible via l'état OTA.
  if (!otaLocalDirectMode) {
    if (otaTotalBytes) {
      if ((p % 5) == 0) otaWebLog(otaStage);
    } else if (otaBytesReceived >= lastOtaLogBytes + 32768UL) {
      lastOtaLogBytes = otaBytesReceived;
      otaWebLog(otaStage);
    }
  }
  return true;
}

bool otaFinishCommon(OTAStreamValidator &validator) {
  String reason;
  if (!validator.finish(reason)) {
    Update.end(false);
    otaLastError = "FIRMWARE INVALIDE";
    otaStage = "Firmware refuse : " + reason;
    otaStatusMessage = otaStage;
    return false;
  }
  if (!Update.end(true) || !Update.isFinished()) {
    otaLastError = "PROGRAMMATION";
    otaStage = "Validation / programmation echouee";
    otaStatusMessage = otaStage;
    return false;
  }
  otaProgressPercent = 100;
  otaBytesReceived = otaTotalBytes;
  otaStage = "Mise a jour terminee - redemarrage";
  otaStatusMessage = otaStage;
  return true;
}

bool otaReadHTTPChunkToUpdate(uint32_t offset, size_t wanted, size_t &received,
                              OTAStreamValidator &validator) {
  received = 0;
  otaFlushModemInput();
  traceOTATX("AT+HTTPREAD=" + String(offset) + "," + String(wanted));
  if (otaWebStreamActive) server.sendContent("AT> AT+HTTPREAD=" + String(offset) + "," + String(wanted) + "\n");
  Serial.print("AT+HTTPREAD="); Serial.print(offset); Serial.print(","); Serial.println(wanted);

  String head; head.reserve(100);
  uint32_t start = millis(); int n = -1;
  while (millis() - start < 120000UL) {
    while (Serial.available()) {
      char c=(char)Serial.read(); head += c;
      if (head.length()>160) head.remove(0, head.length()-160);
      int p=head.lastIndexOf("+HTTPREAD:");
      if (p>=0) {
        int e=head.indexOf('\n',p);
        if(e>=0){ String v=head.substring(p+10,e); v.trim(); n=v.toInt();
          traceOTARX("+HTTPREAD: " + v);
          if(otaWebStreamActive) server.sendContent("AT< +HTTPREAD: "+v+"\n"); break; }
      }
      if(head.indexOf("ERROR")>=0) return false;
    }
    if(n>=0) break; yield();
  }
  if(n==0){ received=0; return true; }
  if(n<0 || (size_t)n>wanted) return false;

  uint8_t buf[MODEM_OTA_CHUNK]; size_t left=(size_t)n;
  uint32_t deadline=millis()+120000UL;
  while(left && (int32_t)(deadline-millis())>0){
    size_t avail=Serial.available(); if(!avail){yield();continue;}
    size_t ask=left; if(ask>sizeof(buf)) ask=sizeof(buf); if(avail<ask) ask=avail;
    size_t got=Serial.readBytes((char*)buf,ask); if(!got){yield();continue;}
    if(!otaWriteCommon(buf,got,validator)) return false;
    received += got; left -= got; yield();
  }
  if(left) return false;

  // A7670 : +HTTPREAD:<n> est suivi directement par les n octets.
  // Ne pas attendre +HTTPREAD: 0 apres chaque bloc. La lecture suivante
  // retournera 0 lorsqu il ne reste plus de donnees.
  return true;
}

// ============================================================
// OTA 4G : meme moteur OTA que le WiFi, seule la source des chunks change.
// ============================================================
// ============================================================
// OTA HTTP SEQUENTIEL POUR A7670E - V9 TRAILER FIX
//
// AT+HTTPACTION=0 est execute UNE SEULE FOIS. Le modem telecharge
// alors la ressource HTTP. L'ESP8266 lit ensuite le contenu deja
// disponible dans le modem avec AT+HTTPREAD=<offset>,<taille>.
//
// Regle retenue :
//   0,1024
//   1024,1024
//   2048,1024
//   3072,1024
//   ...
//
// +HTTPREAD:<n> donne la quantite reellement retournee.
// Si n < 1024 (ou n == 0 apres un dernier bloc exactement multiple
// de 1024), le telechargement est considere termine.
// Aucun Range HTTP et aucun nouvel HTTPACTION entre les lectures.
// ============================================================
// Lecture séquentielle du cache HTTP A76XX.
// IMPORTANT : on ne vide jamais l'UART avant HTTPREAD. Le modem peut déjà
// avoir commencé à transmettre le payload binaire.
// Le validateur ne peut jamais interrompre la réception d'un bloc.
bool otaReadHTTPSequentialToUpdate(uint32_t offset, size_t wanted, size_t &received,
                                   OTAStreamValidator &validator) {
  received = 0;
  if (wanted == 0) return false;

  traceOTATX("AT+HTTPREAD=" + String(offset) + "," + String(wanted));
  if (otaWebStreamActive) {
    server.sendContent("AT> AT+HTTPREAD=" + String(offset) + "," + String(wanted) + "\n");
  }
  Serial.print("AT+HTTPREAD=");
  Serial.print(offset);
  Serial.print(",");
  Serial.println(wanted);

  String head;
  head.reserve(100);
  uint32_t start = millis();
  int n = -1;

  // Attendre uniquement l'en-tête +HTTPREAD:<n>.
  while (millis() - start < 120000UL) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      head += c;
      if (head.length() > 160) head.remove(0, head.length() - 160);

      int p = head.lastIndexOf("+HTTPREAD:");
      if (p >= 0) {
        int e = head.indexOf('\n', p);
        if (e >= 0) {
          String v = head.substring(p + 10, e);
          v.trim();
          n = v.toInt();

          traceOTARX("+HTTPREAD: " + v);
          if (otaWebStreamActive) {
            server.sendContent("AT< +HTTPREAD: " + v + "\n");
          }
          break;
        }
      }

      if (head.indexOf("ERROR") >= 0) return false;
    }

    if (n >= 0) break;
    otaWebPump();
  }

  if (n < 0 || (size_t)n > wanted) {
    otaWebLog("UART RX : longueur HTTPREAD invalide : " + String(n) +
              " octets (demande " + String(wanted) + ")");
    return false;
  }

  otaWebLog("UART RX : +HTTPREAD: " + String(n) +
            " octets annonces dans ce segment");

  // Aucun octet disponible maintenant. Le programme appelant pourra
  // relancer exactement le même offset après un court délai.
  if (n == 0) {
    received = 0;

    // Consommer la fin de réponse HTTPREAD si elle est présente.
    String trailer;
    uint32_t trailerStart = millis();
    while (millis() - trailerStart < 1500UL) {
      while (Serial.available()) {
        char c = (char)Serial.read();
        trailer += c;
        if (trailer.length() > 160) trailer.remove(0, trailer.length() - 160);
        if (trailer.indexOf("+HTTPREAD: 0") >= 0 ||
            trailer.indexOf("OK") >= 0 ||
            trailer.indexOf("ERROR") >= 0) break;
      }
      if (trailer.indexOf("OK") >= 0 || trailer.indexOf("ERROR") >= 0) break;
      otaWebPump();
    }

    if (otaWebStreamActive) {
      trailer.trim();
      if (trailer.length()) server.sendContent("AT< " + trailer + "\n");
    }

    return true;
  }

  uint8_t buf[MODEM_OTA_CHUNK];
  size_t left = (size_t)n;
  const uint32_t readDeadline = millis() + 120000UL;

  while (left && (int32_t)(readDeadline - millis()) > 0) {
    size_t avail = Serial.available();

    if (!avail) {
      otaWebPump();
      continue;
    }

    size_t ask = left;
    if (ask > sizeof(buf)) ask = sizeof(buf);
    if (avail < ask) ask = avail;

    size_t got = Serial.readBytes((char*)buf, ask);
    if (!got) {
      otaWebPump();
      continue;
    }

    // Écriture d'abord. Le validateur est tolérant et ne peut pas couper
    // la réception du bloc UART.
    if (Update.write(buf, got) != got) {
      otaLastError = "ECRITURE OTA";
      otaSetProgress(otaBytesReceived, otaTotalBytes, "Erreur d'ecriture OTA");
      otaWebLog("OTA : Update.write() a refuse " + String(got) + " octets");
      return false;
    }

    // Le validateur ne décide qu'à la fin.
    validator.feed(buf, got);

    otaBytesReceived += (uint32_t)got;
    received += got;
    left -= got;

    uint8_t p = 0;
    if (otaTotalBytes) {
      uint32_t q = (otaBytesReceived * 100UL) / otaTotalBytes;
      if (q > 100) q = 100;
      p = (uint8_t)q;
    }
    otaProgressPercent = p;
    otaStage = otaTotalBytes
             ? ("Téléchargement : " + String(p) + "%")
             : ("Téléchargement : " + String(otaBytesReceived) + " octets");

    otaWebPump();
  }

  if (left != 0 || received != (size_t)n) {
    otaWebLog("UART RX : payload incomplet : " + String(received) +
              "/" + String(n) + " octets");
    return false;
  }

  otaWebLog("UART RX : payload reçu intégralement : " +
            String(received) + " octets");

  // A76XX : le trailer +HTTPREAD: 0 doit être consommé, mais il ne faut
  // surtout pas attendre plusieurs secondes ici. Le manuel indique que,
  // lorsque la réponse dépasse le cache du module, les lectures doivent
  // être faites rapidement. Une longue attente entre deux HTTPREAD peut
  // donc faire déborder le cache et bloquer le téléchargement.
  String endResp;
  bool trailerOK = false;
  uint32_t trailerStart = millis();

  while (millis() - trailerStart < OTA_HTTPREAD_TRAILER_WAIT_MS) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      endResp += c;
      if (endResp.length() > 300) endResp.remove(0, endResp.length() - 300);

      if (endResp.indexOf("+HTTPREAD: 0") >= 0) {
        trailerOK = true;
        break;
      }
      if (endResp.indexOf("ERROR") >= 0) break;
    }

    if (trailerOK || endResp.indexOf("ERROR") >= 0) break;
    otaWebPump();
    delay(1);
  }

  if (endResp.indexOf("ERROR") >= 0) {
    otaWebLog("UART RX : erreur modem après le payload HTTPREAD");
    return false;
  }

  if (otaWebStreamActive && endResp.length()) {
    String logResp = endResp;
    logResp.trim();
    if (logResp.length()) server.sendContent("AT< " + logResp + "\n");
  }

  // L'absence du trailer n'est pas une erreur : le payload demandé a déjà
  // été reçu et écrit intégralement.
  return true;
}


bool updateFirmwareFrom4G(const String &urlInput) {
  String url = normalizeFirmwareURL(urlInput);
  otaLastError = "";
  otaOTAInProgress = false;
  cleanupTemporaryFiles();
  otaSetProgress(0, 0, "Preparation de la mise a jour 4G...");
  otaWebLog("Préparation de la mise à jour...");

  bool ok = false;
  bool httpStarted = false;
  String netResp;
  uint32_t netStart = 0;
  bool netOK = false;
  bool netAlreadyOpen = false;
  String apnCmd;
  String urlCmd;

  if (url.length() < 12 || !(url.startsWith("http://") || url.startsWith("https://"))) {
    otaLastError = "URL INVALIDE";
    otaSetProgress(0, 0, "URL invalide");
    modemOTAActive = false;
    return false;
  }

  if (!ensureModemReadyForOTA()) {
    otaOTAInProgress = false;
    modemOTAActive = false;
    return false;
  }

  modemOTAActive = true;
  otaSetProgress(0, 0, "Modem pret - preparation du telechargement...");
  otaWebLog("Modem prêt -> arrêt des communications GPS/TCP...");
  stopAllCommunicationsForOTA();
  otaFlushModemInput();

  // Bloc unique : aucun goto, donc aucune variable locale franchie par un saut.
  do {
    if (!otaAT("ATE0", "OK", 3000)) { otaLastError = "MODEM"; break; }
    if (!otaAT("AT+CPIN?", "READY", 5000)) { otaLastError = "SIM"; break; }
    if (!otaAT("AT+CEREG?", "OK", 5000)) { otaLastError = "RESEAU"; break; }

    apnCmd = "AT+CGDCONT=1,\"IP\",\"" + cfg.apn + "\"";
    if (!otaAT(apnCmd, "OK", 5000)) { otaLastError = "APN"; break; }

    otaSetProgress(0, 0, "Ouverture du service Internet...");
    otaWebLog("Ouverture du service Internet...");
    otaFlushModemInput();
    if (otaWebStreamActive) server.sendContent("AT> AT+NETOPEN\n");
    Serial.println("AT+NETOPEN");
    netResp = "";
    netResp.reserve(180);
    netStart = millis();
    netOK = false;
    netAlreadyOpen = false;
    while (millis() - netStart < 20000UL) {
      while (Serial.available()) {
        char c = (char)Serial.read();
        netResp += c;
        if (netResp.length() > 300) netResp.remove(0, netResp.length() - 300);
        if (netResp.indexOf("+NETOPEN: 0") >= 0) netOK = true;
        if (netResp.indexOf("Network is already opened") >= 0) netAlreadyOpen = true;
        if (netResp.indexOf("+IP ERROR: Network is already opened") >= 0) netAlreadyOpen = true;
        if (netResp.indexOf("ERROR") >= 0 && !netAlreadyOpen) break;
      }
      if (netOK || netAlreadyOpen) break;
      otaWebPump();
    }
    traceOTARX(netResp);
    if (!netOK && !netAlreadyOpen) { otaLastError = "NETOPEN"; otaSetProgress(0,0,"Service Internet non ouvert"); break; }
    otaSetProgress(0, 0, netAlreadyOpen ? "Service Internet deja ouvert" : "Service Internet ouvert");

    otaAT("AT+HTTPTERM", "OK", 3000);
    if (url.startsWith("https://")) {
      otaSetProgress(0, 0, "Preparation HTTPS...");
      otaAT("AT+CSSLCFG=\"enableSNI\",0,1", "OK", 5000);
      otaAT("AT+CSSLCFG=\"sslversion\",0,4", "OK", 5000);
    }

    otaSetProgress(0, 0, "Initialisation du telechargement...");
    if (!otaAT("AT+HTTPINIT", "OK", 15000)) { otaLastError = "HTTPINIT"; break; }
    httpStarted = true;

    if (url.startsWith("https://")) {
      if (!otaAT("AT+HTTPPARA=\"SSLCFG\",0", "OK", 5000)) {
        otaLastError = "SSLCFG";
        break;
      }
    }

    urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
    if (!otaAT(urlCmd, "OK", 10000)) { otaLastError = "URL MODEM"; break; }
    otaAT("AT+HTTPPARA=\"RECVTO\",120", "OK", 5000);

    // A76XX : READMODE=0 = lecture séquentielle de la réponse.
    // On reste dans le moteur HTTP existant, sans Range et sans nouvel HTTPACTION.
    otaAT("AT+HTTPPARA=\"READMODE\",0", "OK", 5000);

    otaSetProgress(0, 0, "Telechargement 4G - lecture sequentielle du cache modem...");
    otaWebLog("HTTP GET unique : lecture sequentielle A76XX via HTTPREAD");

    {
      OTAStreamValidator validator;
      // La taille Content-Length n'est pas necessaire. On reserve
      // l'espace OTA disponible et le validateur finalise la taille
      // reelle apres le dernier HTTPREAD.
      if (!otaBeginCommon(0, validator)) break;

      // Une seule demande HTTP. Ensuite, uniquement HTTPREAD.
      uint32_t actionDataLen = 0;
      int httpStatus = 0;
      if (!otaAT("AT+HTTPACTION=0", "OK", 5000)) {
        otaLastError = "HTTPACTION";
        break;
      }
      if (!otaGetAction(actionDataLen, httpStatus)) {
        otaLastError = "HTTPACTION";
        otaSetProgress(0, 0, "Reponse HTTP absente");
        break;
      }

      otaWebLog("UART RX : +HTTPACTION: 0," + String(httpStatus) + "," + String(actionDataLen));
      otaWebLog("HTTP status : " + String(httpStatus) + " (200 attendu)");
      otaWebLog("DataLen HTTP : " + String(actionDataLen) + " (informatif uniquement ; HTTPREAD pilote la lecture)");

      if (httpStatus != 200) {
        otaLastError = "HTTP " + String(httpStatus);
        otaSetProgress(0, 0, "Telechargement refuse : HTTP " + String(httpStatus));
        break;
      }

      // IMPORTANT : ne pas bloquer 30 s ici.
      // Le manuel A76XX précise que pour une réponse supérieure au cache,
      // READMODE=0 doit être lu rapidement. La fenêtre de 30 s est donc
      // utilisée uniquement comme temps maximal d'attente lorsque le
      // prochain segment n'est pas encore disponible.
      otaWebLog("OTA : début immédiat des HTTPREAD (fenêtre d'attente modem = " +
                String(OTA_MODEM_DOWNLOAD_WAIT_MS / 1000UL) + " s)");

      uint32_t offset = 0;
      otaWebLog("OTA : début lecture UART séquentielle, bloc demandé = 1024 octets");
      otaWebLog("OTA : offsets prévus = 0, 1024, 2048, 3072... ");
      bool failed = false;
      bool downloadDone = false;
      const size_t readChunk = MODEM_OTA_CHUNK;
      uint32_t noDataStart = 0;
      bool waitingForMoreData = false;

      while (!downloadDone) {
        size_t got = 0;

        otaWebLog("UART TX : AT+HTTPREAD=" + String(offset) + "," + String(readChunk));
        if (!otaReadHTTPSequentialToUpdate(offset, readChunk, got, validator)) {
          otaWebLog("OTA : échec pendant HTTPREAD à l'offset " + String(offset));
          failed = true;
          break;
        }

        if (got == 0) {
          // Aucun bloc disponible à cet instant : ne pas conclure immédiatement
          // que le firmware est terminé. Attendre jusqu'à 30 s en relançant
          // exactement le même offset.
          if (!waitingForMoreData) {
            waitingForMoreData = true;
            noDataStart = millis();
            otaWebLog("OTA : aucun segment disponible -> attente du modem...");
          }

          if (millis() - noDataStart >= OTA_MODEM_DOWNLOAD_WAIT_MS) {
            // Si le serveur a fourni une taille valide dans HTTPACTION,
            // offset >= DataLen confirme la fin.
            if (actionDataLen > 0 && offset >= actionDataLen) {
              otaWebLog("OTA : fin confirmée par DataLen HTTP = " + String(actionDataLen));
              downloadDone = true;
              break;
            }

            otaWebLog("OTA : aucun nouveau segment depuis " +
                      String(OTA_MODEM_DOWNLOAD_WAIT_MS / 1000UL) + " s");
            failed = true;
            otaLastError = "DONNEES HTTP INCOMPLETES";
            break;
          }

          uint32_t wait100 = millis();
          while (millis() - wait100 < 100UL) {
            otaWebPump();
            delay(1);
          }
          continue;
        }

        waitingForMoreData = false;
        otaWebLog("OTA : HTTPREAD a fourni " + String(got) +
                  " octets à partir de l'offset " + String(offset));

        offset += (uint32_t)got;
        otaTotalBytes = offset;
        otaWebLog("OTA : octets cumulés = " + String(offset));
        otaSetProgress(offset, 0, "Telechargement : " + String(offset) + " octets");

        // Si DataLen est fiable, il permet de connaître exactement la fin.
        if (actionDataLen > 0 && offset >= actionDataLen) {
          downloadDone = true;
        } else if (got < readChunk) {
          downloadDone = true;
        } else {
          otaWebLog("OTA : pause " + String(OTA_HTTPREAD_GAP_MS) +
                    " ms avant le prochain HTTPREAD");
          uint32_t gapStart = millis();
          while (millis() - gapStart < OTA_HTTPREAD_GAP_MS) {
            otaWebPump();
            delay(1);
          }
        }

        otaWebPump();
      }

      if (failed || !downloadDone || offset == 0) {
        Update.end(false);
        if (otaLastError.length() == 0) otaLastError = "TELECHARGEMENT";
        otaSetProgress(offset, 0, "Telechargement incomplet");
        break;
      }

      // Le fichier est maintenant entier dans la partition OTA.
      // On passe ensuite au validateur final avec sa taille reelle.
      validator.total = offset;
      validator.totalKnown = true;
      otaTotalBytes = offset;
      otaSetProgress(offset, offset, "Telechargement termine - validation du firmware...");
      otaWebLog("UART : téléchargement terminé après " + String(offset) + " octets");
      otaWebLog("OTA : aucun nouvel HTTPACTION ; passage direct au validateur");

      if (!otaFinishCommon(validator)) break;
      ok = true;
      otaWebLog("Firmware 4G valide : verification OK -> installation autorisee");
    }
  } while (false);

  if (httpStarted) otaAT("AT+HTTPTERM", "OK", 5000);
  cleanupTemporaryFiles();
  modemOTAActive = false;
  if (!ok) otaOTAInProgress = false;
  return ok;
}

// ============================================================
// OTA WiFi : même partition OTA et mêmes fonctions communes que l'OTA 4G.
// Les deux sources écrivent directement avec Update.write(), sans fichier .bin
// intermédiaire dans SPIFFS. Ne jamais appeler server.handleClient() depuis
// le callback de réception HTTP multipart.
// OTA WiFi : meme moteur OTA, seule la source des chunks change.
// ============================================================
bool updateFirmwareFromWiFiURL(const String &url) {
  otaLastError = "";
  otaOTAInProgress = false;
  otaSetProgress(0, 0, "Preparation de la mise a jour WiFi...");

  if (!(url.startsWith("http://") || url.startsWith("https://"))) {
    otaLastError = "URL INVALIDE";
    otaSetProgress(0,0,"URL WiFi invalide");
    return false;
  }

  WiFiClient client;
  HTTPClient http;
  if (!http.begin(client, url)) { otaLastError="HTTP BEGIN"; otaSetProgress(0,0,"Connexion serveur impossible"); return false; }
  otaSetProgress(0,0,"Connexion au serveur WiFi...");
  int code = http.GET();
  if (code != HTTP_CODE_OK) { otaLastError="HTTP " + String(code); otaSetProgress(0,0,"Serveur : HTTP " + String(code)); http.end(); return false; }

  int len = http.getSize();
  uint32_t announcedSize = (len > 0) ? (uint32_t)len : 0;
  if (announcedSize) {
    otaSetProgress(0, announcedSize, "Firmware WiFi : " + String(announcedSize) + " octets");
  } else {
    otaSetProgress(0, 0, "Firmware WiFi : taille non fournie - telechargement en cours");
  }

  OTAStreamValidator validator;
  if (!otaBeginCommon(announcedSize, validator)) { http.end(); return false; }

  WiFiClient *stream = http.getStreamPtr();
  uint8_t buf[MODEM_OTA_CHUNK];
  uint32_t received = 0;
  bool failed = false;
  bool downloadDone = false;
  uint32_t lastData = millis();

  while (!downloadDone) {
    // Correctif : si Content-Length est connu et entièrement reçu, terminer
    // sans attendre la fermeture de la connexion HTTP persistante.
    if (announcedSize > 0 && received >= announcedSize) {
      downloadDone = true;
      break;
    }

    size_t avail = stream->available();
    if (!avail) {
      if (!stream->connected()) { downloadDone = true; break; }
      if (millis() - lastData > 120000UL) { failed = true; break; }
      delay(1); yield(); continue;
    }
    size_t ask = avail;
    if (ask > sizeof(buf)) ask = sizeof(buf);
    if (announcedSize) {
      uint32_t remain = announcedSize - received;
      if (ask > remain) ask = remain;
    }
    size_t n = stream->readBytes((char*)buf, ask);
    if (!n || !otaWriteCommon(buf, n, validator)) { failed = true; break; }
    received += (uint32_t)n;
    lastData = millis();
    otaSetProgress(received, announcedSize, announcedSize ?
                   ("Telechargement WiFi : " + String(otaProgressPercent) + "%") :
                   ("Telechargement WiFi : " + String(received) + " octets"));
    yield();
  }

  bool ok = false;
  if (failed || (announcedSize > 0 && received != announcedSize)) {
    Update.end(false);
    otaLastError = "TELECHARGEMENT";
    otaSetProgress(received, announcedSize, "Telechargement WiFi incomplet");
  } else {
    if (!announcedSize) { validator.total = received; validator.totalKnown = true; otaTotalBytes = received; }
    ok = otaFinishCommon(validator);
  }

  http.end();
  otaOTAInProgress = false;
  return ok;
}

void handleUpdateSMSCommand(const String &rawCommand) {
  // Format: UPDATE_motdepasse_VERSION
  if (!rawCommand.startsWith("UPDATE_")) return;

  int p1 = rawCommand.indexOf('_', 7);
  if (p1 < 0) {
    smsReply("MISE A JOUR\nDEMANDE : FORMAT INVALIDE");
    return;
  }

  String pass = rawCommand.substring(7, p1);
  String version = rawCommand.substring(p1 + 1);
  pass.trim();
  version.trim();

  if (!verifySMSAccess(pass)) return;

  // L'utilisateur fournit seulement la version/nom du firmware.
  // Le programme construit automatiquement l'URL complète et ajoute .bin.
  String url = buildGitHubFirmwareURL(version);
  if (!url.length()) {
    smsReply("MISE A JOUR\nDEMANDE : VERSION INVALIDE");
    return;
  }

  // Refuser une seconde demande pendant qu'une OTA SMS est déjà planifiée.
  if (smsOTARequestPending || modemOTAActive) {
    smsReply("MISE A JOUR\nDEMANDE : DEJA EN COURS");
    return;
  }

  // Le SMS de confirmation est mis en file immédiatement.
  smsReply("MISE A JOUR\nDEMANDE : RECUE\nTRAITEMENT : EN COURS");

  // Sauvegarder l'expéditeur avant que processModemLine() ne réinitialise
  // smsSender. L'OTA sera lancée depuis loop(), pas depuis le parser SMS.
  smsOTAReplyNumber = smsSender;
  smsOTARequestURL = url;
  smsOTARequestPending = true;
  smsOTAStartNotBefore = millis() + 1500UL;
}

// Lance l'OTA SMS hors du contexte de traitement de la ligne modem.
// Ainsi taskModem() / taskNetwork() peuvent évoluer normalement avant OTA.
void taskPendingSMSOTA() {
  if (!smsOTARequestPending || modemOTAActive) return;
  if ((long)(millis() - smsOTAStartNotBefore) < 0) return;

  // L'accusé de réception doit être sorti avant de couper les communications
  // et de fermer les sessions TCP/HTTP.
  if (smsTxBusy || smsQueueCount > 0) return;
  if (modemWaitingResponse || modemTcpBusy || modemExpectSendPrompt) return;

  String url = smsOTARequestURL;
  String replyNumber = smsOTAReplyNumber;

  smsOTARequestURL = "";
  smsOTAReplyNumber = "";
  smsOTARequestPending = false;

  bool ok = updateFirmwareFrom4G(url);

  if (ok) {
    smsSend(replyNumber,
      "MISE A JOUR\nRESULTAT : REUSSIE\nVERSION : " + String(NUMO_VERSION) +
      "\nREDÉMARRAGE : OUI");

    uint32_t waitStart = millis();
    while ((smsTxBusy || smsQueueCount > 0) &&
           millis() - waitStart < 15000UL) {
      taskModem();
      yield();
    }

    delay(500);
    ESP.restart();
  } else {
    smsSend(replyNumber,
      "MISE A JOUR\nRESULTAT : ECHEC\nETAPE : " + otaLastError +
      "\nVERSION : " + String(NUMO_VERSION));
  }
}

void handleSMSCommand(String command) {
  command.trim();
  if (command.startsWith("UPDATE_")) { handleUpdateSMSCommand(command); return; }
  command.replace("\r", "");
  command.replace("\n", " ");
  command.trim();

  String body, password;
  if (!splitLastToken(command, body, password)) {
    smsNoReply();
    return;
  }

  // Gestion des numeros : reservee a un administrateur deja autorise
  // (SMS avec le mot de passe principal) ou a une commande serveur authentifiee.
  String commandUpper = body;
  commandUpper.toUpperCase();
  bool numberAdminCommand = commandUpper.startsWith("ADMIN1 ") || commandUpper.startsWith("ADMIN2 ") ||
                            commandUpper.startsWith("ADMIN3 ") || commandUpper == "ADMINS?" ||
                            commandUpper.startsWith("MQTTREPLY ") || commandUpper == "MQTTREPLY?";
  if (numberAdminCommand) {
    bool allowed = remoteCommandContext || (smsAuthorized(smsSender) && verifySMSPassword(password));
    if (!allowed) {
      addLog("SMS number config rejected");
      return;
    }
    if (commandUpper == "ADMINS?") {
      smsReply("ADMINS\n1:" + (cfg.smsAdmin1.length() ? cfg.smsAdmin1 : "OFF") +
               "\n2:" + (cfg.smsAdmin2.length() ? cfg.smsAdmin2 : "OFF") +
               "\n3:" + (cfg.smsAdmin3.length() ? cfg.smsAdmin3 : "OFF"));
      return;
    }
    if (commandUpper == "MQTTREPLY?") {
      smsReply("MQTTREPLY:" + (cfg.mqttReplyNumber.length() ? cfg.mqttReplyNumber : "ADMIN1/FALLBACK"));
      return;
    }
    int sp = body.indexOf(' ');
    String key = body.substring(0, sp);
    key.toUpperCase();
    String value = body.substring(sp + 1);
    value.trim();
    value = normalizePhone(value);
    if (key == "MQTTREPLY") {
      if (value.equalsIgnoreCase("OFF")) cfg.mqttReplyNumber = "";
      else if (!validPhoneNumber(value)) { smsReply("ERREUR: NUMERO INVALIDE"); return; }
      else cfg.mqttReplyNumber = value;
      saveConfig();
      smsReply("MQTTREPLY:" + (cfg.mqttReplyNumber.length() ? cfg.mqttReplyNumber : "OFF") + "\nRESULTAT:OK");
      return;
    }
    if (value.equalsIgnoreCase("OFF")) value = "";
    else if (!validPhoneNumber(value)) { smsReply("ERREUR: NUMERO INVALIDE"); return; }
    if (key == "ADMIN1") cfg.smsAdmin1 = value;
    else if (key == "ADMIN2") cfg.smsAdmin2 = value;
    else if (key == "ADMIN3") cfg.smsAdmin3 = value;
    saveConfig();
    smsReply(key + ":" + (value.length() ? value : "OFF") + "\nRESULTAT:OK");
    return;
  }

  // Le mot de passe DOIT être le dernier champ.
  if (!verifySMSAccess(password)) {
    addLog("SMS rejected: bad credential");
    return;
  }

  String u = body;
  u.toUpperCase();
  addLog("SMS CMD AUTH: " + u);

  if (u == "HELP" || u == "AIDE" || u == "COMMANDES") {
    smsReply("AIDE\nSTATUS_mdp LOCATION_mdp BATTERY_mdp CONFIG?_mdp\nMODE_x_mdp GPSINT_n_mdp SENDINT_n_mdp STOPINT_n_mdp LONGINT_n_mdp VIBTIME_n_mdp VIBTIME?_mdp HEARTBEAT_n_mdp\nGPS ON_mdp GPS OFF_mdp GPS STATUS_mdp MODEM ON_mdp MODEM OFF_mdp MODEM STATUS_mdp\nSMS ON_mdp SMS OFF_mdp SMS STATUS_mdp SERVER1 ON_mdp SERVER1 OFF_mdp SERVER1 STATUS_mdp\nSERVER2 ON_mdp SERVER2 OFF_mdp SERVER2 STATUS_mdp MQTT ON_mdp MQTT OFF_mdp MQTT STATUS_mdp\nWIFI ON_mdp WIFI OFF_mdp WIFI STATUS_mdp LED ON_mdp LED OFF_mdp LED STATUS_mdp\nAPN_x_mdp SERVER1_h_p_mdp SERVER2_h_p_mdp ADMIN1_numero_mdp ADMIN2_numero_mdp ADMIN3_numero_mdp\nSAVE_mdp RESTART_mdp FACTORY_mdp UPDATE_mdp_version");
    return;
  }

  if (u == "STATUS" || u == "INFO") {
    String modeName =
      cfg.energyMode == 0 ? "NORMAL" :
      cfg.energyMode == 1 ? "ECO" :
      cfg.energyMode == 2 ? "PERFORMANCE" :
      cfg.energyMode == 3 ? "ULTRA ECO" :
      cfg.energyMode == 4 ? "TEST" : "PERMANENCE";
    String r = "NUMOTRONIC-VER:" + String(NUMO_VERSION) + "\n";
    r +=" IMEI:" + cfg.imei + "\n";
    r += "BAT:" + String(batteryVoltage, 2) + "V \n";
    r += "GPS:" + String(gps.fix ? "FIX" : "NO FIX") + "\n";
    r += "MODE:" + modeName +  "\n";
    r += "BUF:" + String(bufferPending) ;
    smsReply("" + r);
    return;
  }

  if (u == "LOCATION" || u == "GPS") {
    if (gps.fix) {
      smsReply("" + locationSMS());
    } else {
      // Une demande de localisation reste active jusqu'a l'obtention
      // d'un FIX valide, meme si le tracker se rendort entre-temps.
      savePendingLocationRequest(smsSender, mqttCommandContext);
      // La demande reste en attente et sera traitee uniquement au prochain CHECK.
      setGPSPower(false);
      String r = "LOCALISATION\nDEMANDE RECUE\nGPS : RECHERCHE";
      smsReply("" + r);
    }
    return;
  }

  if (u == "BATTERY" || u == "BAT") {
    String r = "BATTERIE\n";
    r += String(batteryVoltage, 2) + "V / " + String(batteryPercent) + "%\n";
    r += batteryStatus();
    smsReply("" + r);
    return;
  }

  // Contrôle distant étendu : mêmes commandes utilisables par SMS, TCP et MQTT.
  if (u == "GPS ON") { setGPSPower(true); smsReply("GPS\nETAT : ACTIF\nRESULTAT : OK"); return; }
  if (u == "GPS OFF") { setGPSPower(false); smsReply("GPS\nETAT : ARRETE\nRESULTAT : OK"); return; }
  if (u == "GPS STATUS" || u == "GPS?") { smsReply(String("GPS\nETAT : ") + (gpsPower ? "ACTIF" : "ARRETE") + "\nFIX : " + (gps.fix ? "OUI" : "NON")); return; }

  if (u == "MODEM ON") { setModemPower(true); smsReply("COMMUNICATION\nETAT : ACTIVEE\nRESULTAT : OK"); return; }
  if (u == "MODEM STATUS" || u == "MODEM?") { smsReply(String("COMMUNICATION\nETAT : ") + (modemPower ? "ACTIVE" : "ARRETEE") + "\nRESEAU : " + (networkRegistered ? "OK" : "NON")); return; }
  if (u == "MODEM OFF") {
    smsReply("COMMUNICATION\nARRET DEMANDE\nRESULTAT : OK");
    remoteModemOffPending = true;
    return;
  }

  if (u == "SMS ON") { cfg.smsEnable = true; saveConfig(); smsReply("SMS\nETAT : ACTIF\nRESULTAT : OK"); return; }
  if (u == "SMS OFF") { cfg.smsEnable = false; saveConfig(); smsReply("SMS\nETAT : ARRETE\nRESULTAT : OK"); return; }
  if (u == "SMS STATUS" || u == "SMS?") { smsReply(String("SMS\nETAT : ") + (cfg.smsEnable ? "ACTIF" : "ARRETE")); return; }

  if (u == "SERVER1 ON") { cfg.server1Enable = true; saveConfig(); initLinks(); smsReply("SERVEUR 1\nETAT : ACTIF\nRESULTAT : OK"); return; }
  if (u == "SERVER1 OFF") { cfg.server1Enable = false; saveConfig(); initLinks(); smsReply("SERVEUR 1\nETAT : ARRETE\nRESULTAT : OK"); return; }
  if (u == "SERVER1 STATUS" || u == "SERVER1?") { smsReply(String("SERVEUR 1\nETAT : ") + (cfg.server1Enable ? "ACTIF" : "ARRETE") + "\nLIEN : " + (link1.connected ? "CONNECTE" : "DECONNECTE")); return; }

  if (u == "SERVER2 ON") { cfg.server2Enable = true; saveConfig(); initLinks(); smsReply("SERVEUR 2\nETAT : ACTIF\nRESULTAT : OK"); return; }
  if (u == "SERVER2 OFF") { cfg.server2Enable = false; saveConfig(); initLinks(); smsReply("SERVEUR 2\nETAT : ARRETE\nRESULTAT : OK"); return; }
  if (u == "SERVER2 STATUS" || u == "SERVER2?") { smsReply(String("SERVEUR 2\nETAT : ") + (cfg.server2Enable ? "ACTIF" : "ARRETE") + "\nLIEN : " + (link2.connected ? "CONNECTE" : "DECONNECTE")); return; }

  if (u == "MQTT ON") { cfg.mqttEnable = true; saveConfig(); smsReply("SERVICE DISTANT\nETAT : ACTIF\nRESULTAT : OK"); return; }
  if (u == "MQTT OFF") { cfg.mqttEnable = false; mqttConnected = false; mqttSubscribed = false; saveConfig(); smsReply("SERVICE DISTANT\nETAT : ARRETE\nRESULTAT : OK"); return; }
  if (u == "MQTT STATUS" || u == "MQTT?") { smsReply(String("SERVICE DISTANT\nETAT : ") + (cfg.mqttEnable ? "ACTIF" : "ARRETE") + "\nLIEN : " + (mqttConnected && mqttSubscribed ? "CONNECTE" : "EN ATTENTE")); return; }

  // Contrôle Wi-Fi par SMS : WIFI ON / WIFI OFF / WIFI STATUS.
  if (u == "WIFI ON") {
    startWiFi();
    smsReply("WIFI\nETAT : ON\nIP : " + WiFi.softAPIP().toString());
    return;
  }
  if (u == "WIFI OFF") {
    stopWiFi();
    smsReply("WIFI\nETAT : OFF");
    return;
  }
  if (u == "WIFI STATUS" || u == "WIFI?") {
    String r = "WI-FI\n";
    r += "ETAT : " + String(wifiEnabled ? "ACTIF" : "ARRETE") + "\n";
    r += "ADRESSE : " + String(wifiEnabled ? WiFi.softAPIP().toString() : "OFF") + "\n";
    r += "APPAREILS : " + String(wifiEnabled ? WiFi.softAPgetStationNum() : 0);
    smsReply(r);
    return;
  }

  if (u == "LED ON") { cfg.ledEnable = true; saveConfig(); smsReply("LED\nETAT : ON\nRESULTAT : OK"); return; }
  if (u == "LED OFF") { cfg.ledEnable = false; saveConfig(); smsReply("LED\nETAT : OFF\nRESULTAT : OK"); return; }
  if (u == "LED STATUS" || u == "LED?") { smsReply(String("LED\nETAT : ") + (cfg.ledEnable ? "ON" : "OFF")); return; }

  if (u == "MODE NORMAL") { cfg.energyMode = 0; saveConfig(); smsReply("MODE\nNORMAL\nRESULTAT : OK"); return; }
  if (u == "MODE ECO") { cfg.energyMode = 1; saveConfig(); smsReply("MODE\nECO\nRESULTAT : OK"); return; }
  if (u == "MODE PERFORMANCE") { cfg.energyMode = 2; saveConfig(); smsReply("MODE\nPERFORMANCE\nRESULTAT : OK"); return; }
  if (u == "MODE ULTRA ECO") { cfg.energyMode = 3; saveConfig(); smsReply("MODE\nULTRA ECO\nRESULTAT : OK"); return; }
  if (u == "MODE TEST") { cfg.energyMode = 4; saveConfig(); smsReply("MODE\nTEST\nRESULTAT : OK"); return; }
  if (u == "MODE PERMANENCE") { cfg.energyMode = 5; saveConfig(); smsReply("MODE\nPERMANENCE\nRESULTAT : OK"); return; }

  if (u == "BUFFER ON" || u == "BUFFER OFF") { cfg.bufferEnable = true; saveConfig(); smsReply("BUFFER=ON;STOCKAGE=PERMANENT"); return; }
  if (u == "BUFFER STATUS" || u == "BUFFER?") {
    String r = "MEMOIRE DES TRAMES\n";
    r += "ETAT : ACTIVE\n";
    r += "EN ATTENTE : " + String(bufferPending) + "\n";
    r += "CAPACITE : " + String(cfg.bufferMax) + "\n";
    r += "RECUPERATION : " + String(cfg.recoveryRate) + " éléments/s";
    smsReply(r);
    return;
  }
  if (u == "MODE?") {
    String modeName =
      cfg.energyMode == 0 ? "NORMAL" :
      cfg.energyMode == 1 ? "ECO" :
      cfg.energyMode == 2 ? "PERFORMANCE" :
      cfg.energyMode == 3 ? "ULTRA ECO" :
      cfg.energyMode == 4 ? "TEST" : "PERMANENCE";
    smsReply("MODE:" + modeName);
    return;
  }

  if (u.startsWith("GPSINT ")) {
    uint16_t v = u.substring(7).toInt();
    if (v >= 1 && v <= 60) {
      cfg.gpsInterval = v; saveConfig();
      smsReply("GPSINT:" + String(v) + "s");
    }
    return;
  }

  if (u == "GPSINT?") {
    String r = "CONFIGURATION GPS\n";
    r += "LECTURE GPS : " + String(cfg.gpsInterval) + " s\n";
    r += "POSITION : " + String(cfg.positionInterval) + " s\n";
    r += "GRAND INTERVALLE : " + String(cfg.longIntervalMin) + " min";
    smsReply(r);
    return;
  }

  if (u.startsWith("SENDINT ")) {
    uint16_t v = u.substring(8).toInt();
    if (v >= 1 && v <= 3600) {
      cfg.positionInterval = v; saveConfig();
      smsReply("SENDINT:" + String(v) + "s");
    }
    return;
  }

  if (u == "SENDINT?") {
    String r = "CONFIGURATION POSITION\n";
    r += "INTERVALLE : " + String(cfg.positionInterval) + " s\n";
    r += "GRAND INTERVALLE : " + String(cfg.longIntervalMin) + " min";
    smsReply(r);
    return;
  }

  if (u.startsWith("STOPINT ")) {
    uint16_t v = u.substring(8).toInt();
    if (v >= 5 && v <= 3600) {
      cfg.stopDelay = v; saveConfig();
      smsReply("STOPINT:" + String(v) + "s");
    }
    return;
  }

  if (u == "STOPINT?") {
    smsReply("STOPINT:" + String(cfg.stopDelay) + "s");
    return;
  }

  if (u.startsWith("VIBTIME ")) {
    int v = u.substring(8).toInt();
    if (v >= 1 && v <= 3600) {
      cfg.vibrationWatchSeconds = (uint16_t)v;
      saveConfig();
      smsReply("VIBTIME:" + String(cfg.vibrationWatchSeconds) + "s\\nRESULTAT:OK");
    } else {
      smsReply("ERREUR: VIBTIME entre 1 et 3600 s");
    }
    return;
  }

  if (u == "VIBTIME?") {
    smsReply("VIBTIME:" + String(cfg.vibrationWatchSeconds) + "s");
    return;
  }

  if (u.startsWith("LONGINT ")) {
    uint32_t v = (uint32_t)u.substring(8).toInt();
    if (v >= 1 && v <= 10080) {
      cfg.longIntervalMin = v; saveConfig();
      smsReply("LONGINT:" + String(v) + "min");
    }
    return;
  }

  if (u == "LONGINT?") {
    smsReply("LONGINT:" + String(cfg.longIntervalMin) + "min");
    return;
  }

  if (u.startsWith("HEARTBEAT ")) {
    uint16_t v = u.substring(10).toInt();
    if (v >= 30 && v <= 3600) {
      cfg.heartbeatInterval = v; saveConfig();
      smsReply("HEARTBEAT:" + String(v) + "s");
    }
    return;
  }

  if (u == "HEARTBEAT?") {
    smsReply("HEARTBEAT:" + String(cfg.heartbeatInterval) + "s");
    return;
  }

  if (u.startsWith("APN ")) {
    String v = body.substring(4); v.trim();
    if (v.length() > 0 && v.length() < 80) { cfg.apn = v; saveConfig(); smsReply("RESEAU MOBILE\nAPN : ENREGISTRE\nRESULTAT : OK"); }
    return;
  }

  if (u == "APN?") { smsReply("RESEAU MOBILE\nAPN : " + cfg.apn); return; }

  if (u.startsWith("SERVER1 ")) {
    String v = body.substring(8); v.trim();
    int sep = v.lastIndexOf(' ');
    if (sep > 0) {
      String host = v.substring(0, sep);
      uint16_t port = v.substring(sep + 1).toInt();
      if (host.length() && port > 0) {
        cfg.server1 = host; cfg.port1 = port; cfg.server1Enable = true; saveConfig();
        smsReply("SERVEUR 1\nCHANGEMENT : OK\n" + cfg.server1 + ":" + String(cfg.port1));
      }
    }
    return;
  }

  if (u == "SERVER1?") {
    smsReply("SERVEUR 1\nADRESSE : " + cfg.server1 + "\nPORT : " + String(cfg.port1));
    return;
  }

  if (u.startsWith("SERVER2 ")) {
    String v = body.substring(8); v.trim();
    int sep = v.lastIndexOf(' ');
    if (sep > 0) {
      String host = v.substring(0, sep);
      uint16_t port = v.substring(sep + 1).toInt();
      if (host.length() && port > 0) {
        cfg.server2 = host; cfg.port2 = port; cfg.server2Enable = true; saveConfig();
        smsReply("SERVEUR 2\nCHANGEMENT : OK\n" + cfg.server2 + ":" + String(cfg.port2));
      }
    }
    return;
  }

  if (u == "SERVER2?") {
    smsReply("SERVEUR 2\nADRESSE : " + cfg.server2 + "\nPORT : " + String(cfg.port2));
    return;
  }

  if (u == "CONFIG?") {
    String modeName =
      cfg.energyMode == 0 ? "NORMAL" :
      cfg.energyMode == 1 ? "ECO" :
      cfg.energyMode == 2 ? "PERFORMANCE" :
      cfg.energyMode == 3 ? "ULTRA ECO" :
      cfg.energyMode == 4 ? "TEST" : "PERMANENCE";
    String r = "CONFIG\n";
    r += "ID:" + cfg.deviceID + "\n";
    r += "GPS:" + String(cfg.gpsInterval) + "s POS:" + String(cfg.positionInterval) + "s\n";
    r += "STOP:" + String(cfg.stopDelay) + "s CHECK:" + String(cfg.longIntervalMin) + "min\n";
    r += "S1:" + cfg.server1 + ":" + String(cfg.port1) + "\n";
    r += "S2:" + cfg.server2 + ":" + String(cfg.port2) + "\n";
    r += "BUF:" + String(cfg.bufferMax) + " MODE:" + modeName;
    smsReply(r);
    return;
  }

  if (u == "SAVE") { saveConfig(); smsReply("SAUVEGARDE\nRESULTAT : OK"); return; }

  // Changer le mot de passe : PASS nouveau_mdp ancien_mdp
  // L'ancien mot de passe est toujours le dernier champ du SMS.
  if (u.startsWith("PASS ")) {
    String newPass = body.substring(5); newPass.trim();
    if (newPass.length() >= 4 && newPass.length() <= 16 && newPass.indexOf(' ') < 0 && newPass.indexOf('_') < 0) {
      cfg.smsPassword = newPass;
      saveConfig();
      smsReply("PASS:OK");
    }
    return;
  }

  if (u == "RESTART") {
    smsReply("REDÉMARRAGE\nRESULTAT : OK");
    delay(500);
    ESP.restart();
    return;
  }

  if (u == "FACTORY" || u == "FACTORYRESET" || u == "RESETFACTORY") {
    smsReply("VALEURS USINE\nDEMANDE : RECUE\nRESULTAT : EN COURS");
    uint32_t waitStart = millis();
    while ((smsTxBusy || smsQueueCount > 0) && millis() - waitStart < 10000UL) {
      taskModem();
      yield();
    }
    factoryReset();
    delay(500);
    ESP.restart();
    return;
  }

  smsReply("CMD? AIDE_<mot_de_passe>");
}

void parseSMSHeader(const String &line) {
  int q1 = line.indexOf('"');
  int q2 = line.indexOf('"', q1 + 1);
  if (q1 >= 0 && q2 > q1) {
    smsSender = line.substring(q1 + 1, q2);
    smsWaitingBody = true;
  }
}

void processModemLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (smsTxBusy && smsTxStage == 4) {
    String up=line; up.toUpperCase();
    if (up.startsWith("+CMGS:") || up == "OK") {
      smsTxComplete(true);
      return;
    }
    if (up == "ERROR" || up.indexOf("CMS ERROR") >= 0) {
      smsTxComplete(false);
      return;
    }
  }

  // URC MQTT : réception d'une commande retained ou d'une nouvelle commande.
  if (line.startsWith("+CMQTTRXSTART:")) {
    mqttRxStage = 1;
    mqttRxTopic = "";
    mqttRxPayload = "";
    mqttRxExpected = 0;
  } else if (line.startsWith("+CMQTTRXTOPIC:")) {
    mqttRxStage = 2;
  } else if (line.startsWith("+CMQTTRXPAYLOAD:")) {
    int c = line.lastIndexOf(',');
    if (c >= 0) mqttRxExpected = (uint16_t)line.substring(c + 1).toInt();
    mqttRxStage = 3;
  } else if (line.startsWith("+CMQTTRXEND:")) {
    traceModemRX("[SERVEUR CMD] MQTT RX END");
    mqttRxStage = 0;
  } else if (line.startsWith("+CMQTTRECV:")) {
    // Certaines versions A76XX utilisent la forme compacte +CMQTTRECV.
    int q1=line.indexOf('"'); int q2=(q1>=0)?line.indexOf('"',q1+1):-1;
    int q3=(q2>=0)?line.indexOf('"',q2+1):-1; int q4=(q3>=0)?line.indexOf('"',q3+1):-1;
    if(q1>=0 && q2>q1 && q3>=0 && q4>q3) {
      String topic=line.substring(q1+1,q2);
      String payload=line.substring(q3+1,q4);
      if(topic==mqttCmdTopic()) mqttHandleCommand(payload);
    }
  } else if (mqttRxStage == 2) {
    mqttRxTopic = line;
    traceModemRX("[SERVEUR CMD] MQTT RX TOPIC: "+mqttRxTopic);
    mqttRxStage = 0;
  } else if (mqttRxStage == 3) {
    mqttRxPayload = line;
    traceModemRX("[SERVEUR CMD] MQTT RX PAYLOAD: "+mqttRxPayload);
    mqttRxStage = 0;
    if (mqttRxTopic == mqttCmdTopic()) mqttHandleCommand(mqttRxPayload);
  }

  if (line.startsWith("+CMQTTCONNECT:")) {
    mqttConnecting = false;
    if (line.endsWith(",0")) { mqttConnected = true; mqttSubscribed = false; }
    else { mqttConnected = false; mqttSubscribed = false; }
  }
  if (line.startsWith("+CMQTTCONNLOST:") || line.startsWith("+CMQTTNONET")) {
    traceModemRX("[SERVEUR CMD] MQTT: CONNEXION PERDUE");
    mqttConnected = false;
    mqttSubscribed = false;
    mqttConnecting = false;
  }
  if (line.startsWith("+CMQTTDISC:") || line.startsWith("+CMQTTSTOP:")) {
    mqttConnected = false; mqttSubscribed = false; mqttServiceStarted = false;
  }

  processGPS103ServerLogic(line);


if (line.startsWith("+CMT:")) {
    // Chaque SMS reçu réinitialise le timeout de communication.
    if (communicationWakeStartedAt != 0 &&
        communicationWakeWindowMs > 0) {
        communicationWakeStartedAt = millis();
        addLog("SMS RX : TIMEOUT MODEM REINITIALISE");
    }

    parseSMSHeader(line);
    return;
}


  if (smsWaitingBody) {
    smsWaitingBody = false;
    if (cfg.smsEnable) handleSMSCommand(line);
    else addLog("SMS ignored: disabled");
    smsSender = "";
  }
}


// ============================================================
// MQTT - SERVEUR CMD
// A76XX MQTT AT : utilisé uniquement pour les commandes.
// La logique TCP des trames GPS reste indépendante.
// ============================================================
bool mqttWaitFor(const String &wanted, uint32_t timeoutMs, String *capture = nullptr) {
  String r;
  String incomingLine;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      r += c;
      if (r.length() > 900) r.remove(0, r.length() - 900);

      // Pendant une attente d'une commande de communication, le modem peut
      // recevoir un SMS. Ne jamais jeter cette information : traiter au moins
      // les URC SMS ici pour que la communication distante ne bloque pas SMS.
      if (c == '\n') {
        incomingLine.trim();
        if (incomingLine.length()) {
          // IMPORTANT : pendant une transaction AT MQTT (CONNECT/SUB/PUB),
          // le modem peut envoyer en parallèle un URC MQTT de réception.
          // Si on ne le transmet pas à processModemLine(), la commande
          // reçue est simplement consommée par mqttWaitFor() et perdue.
          // Le traitement de la commande reste différé pour la publication
          // de RECU, donc aucune transaction MQTT imbriquée n'est lancée ici.
          if (incomingLine.startsWith("+CMQTTRXSTART:") ||
              incomingLine.startsWith("+CMQTTRXTOPIC:") ||
              incomingLine.startsWith("+CMQTTRXPAYLOAD:") ||
              incomingLine.startsWith("+CMQTTRXEND:") ||
              incomingLine.startsWith("+CMQTTRECV:") ||
              incomingLine.startsWith("+CMQTTCONNLOST:") ||
              incomingLine.startsWith("+CMQTTNONET")) {
            processModemLine(incomingLine);
          } else if (incomingLine.startsWith("+CMT:")) {
            parseSMSHeader(incomingLine);
          } else if (smsWaitingBody) {
            smsWaitingBody = false;
            if (cfg.smsEnable) {
              handleSMSCommand(incomingLine);
            } else {
              addLog("SMS ignored: disabled");
            }
            smsSender = "";
          }
        }
        incomingLine = "";
      } else if (c != '\r') {
        if (incomingLine.length() < 320) incomingLine += c;
        else incomingLine = "";
      }

      if (wanted.length() && r.indexOf(wanted) >= 0) {
        if (capture) *capture = r;
        return true;
      }
      if (r.indexOf("ERROR") >= 0) {
        if (capture) *capture = r;
        return false;
      }
    }
    yield();
  }
  if (capture) *capture = r;
  return false;
}

bool mqttAT(const String &cmd, const String &wanted, uint32_t timeoutMs) {
  traceModemTX("[SERVEUR CMD] "+cmd);
  Serial.print(cmd); Serial.print("\r");
  String r;
  bool ok=mqttWaitFor(wanted,timeoutMs,&r);
  r.trim();
  if(r.length()) traceModemRX("[SERVEUR CMD] "+r);
  return ok;
}

bool mqttInput(const String &cmd, const String &data, uint32_t timeoutMs=10000UL) {
  traceModemTX("[SERVEUR CMD] "+cmd);
  Serial.print(cmd); Serial.print("\r");
  if(!mqttWaitFor(">",5000UL)) return false;
  traceModemTX("[SERVEUR CMD DATA] "+data);
  Serial.print(data);
  return mqttWaitFor("OK",timeoutMs);
}

void mqttResetState() {
  mqttServiceStarted=false;
  mqttConnected=false;
  mqttSubscribed=false;
  mqttConnecting=false;
  mqttClientAcquired=false;
  mqttRxStage=0;
  mqttRxExpected=0;
  mqttRxTopic="";
  mqttRxPayload="";
  mqttAckPending=false;
  mqttPendingAck="";
  mqttResponsePending=false;
  mqttPendingResponse="";
  mqttCommandPending=false;
  mqttPendingCommand="";
  mqttCommandContext=false;
  remoteCommandContext=false;
  mqttCommandResponse="";
}

// Le TCP GPS103 utilise les sockets 0 et 1.
// MQTT utilise le client MQTT 1 : il ne partage donc plus l'identifiant 0
// avec le premier lien TCP. Les deux espaces d'index sont distincts dans
// le modem, mais cette séparation évite toute ambiguite dans le programme.
#define MQTT_CLIENT_INDEX 0

void mqttStopService() {
  if (!modemPower) { mqttResetState(); return; }
  if (mqttConnected) mqttAT("AT+CMQTTDISC="+String(MQTT_CLIENT_INDEX)+",10","+CMQTTDISC: "+String(MQTT_CLIENT_INDEX)+",0",5000UL);
  if (mqttServiceStarted) {
    mqttAT("AT+CMQTTREL="+String(MQTT_CLIENT_INDEX),"OK",5000UL);
    mqttAT("AT+CMQTTSTOP","+CMQTTSTOP: 0",15000UL);
  }
  mqttResetState();
}

bool mqttConnectAndSubscribe() {
  if(!modemPower || !cfg.mqttEnable || !cfg.mqttServer.length()) return false;
  if(!modemDataReady) return false;
  if(modemTcpBusy || modemExpectSendPrompt || smsTxBusy || modemWaitingResponse) return false;

  traceModemTX("[SERVEUR CMD] MQTT: START");

  // 1) START : le service MQTT doit etre demarre avant toute autre commande MQTT.
  if(!mqttServiceStarted) {
    if(!mqttAT("AT+CMQTTSTART","+CMQTTSTART: 0",15000UL)) {
      traceModemRX("[SERVEUR CMD] MQTT: START ECHEC");
      return false;
    }
    mqttServiceStarted=true;
    traceModemRX("[SERVEUR CMD] MQTT: START OK");
  }

  // 2) ACCQ : ne jamais refaire ACCQ a chaque tentative de reconnexion.
  String client=cfg.imei.length()==15 ? cfg.imei : cfg.deviceID;
  if(!mqttClientAcquired) {
    traceModemTX("[SERVEUR CMD] MQTT: ACCQ");
    String accq="AT+CMQTTACCQ="+String(MQTT_CLIENT_INDEX)+",\""+client+"\"";
    if(!mqttAT(accq,"OK",7000UL)) {
      traceModemRX("[SERVEUR CMD] MQTT: ACCQ ECHEC");
      return false;
    }
    mqttClientAcquired=true;
    traceModemRX("[SERVEUR CMD] MQTT: ACCQ OK");
  }

  // 3) Parametres MQTT facultatifs/compatibilite.
  mqttAT("AT+CMQTTCFG=\"argtopic\","+String(MQTT_CLIENT_INDEX)+",1,1","OK",5000UL);
  mqttAT("AT+CMQTTCFG=\"version\","+String(MQTT_CLIENT_INDEX)+",4","OK",5000UL);

  // 4) CONNECT.
  traceModemTX("[SERVEUR CMD] MQTT: CONNECT "+cfg.mqttServer+":"+String(cfg.mqttPort));
  String url="tcp://"+cfg.mqttServer+":"+String(cfg.mqttPort);
  String cmd="AT+CMQTTCONNECT="+String(MQTT_CLIENT_INDEX)+",\""+url+"\","+String(cfg.mqttKeepAlive)+",0";
  if(cfg.mqttUser.length()) cmd += ",\""+cfg.mqttUser+"\",\""+cfg.mqttPassword+"\"";

  if(!mqttAT(cmd,"+CMQTTCONNECT: "+String(MQTT_CLIENT_INDEX)+",0",40000UL)) {
    mqttConnected=false;
    mqttSubscribed=false;
    mqttConnecting=false;
    traceModemRX("[SERVEUR CMD] MQTT: CONNECT ECHEC");
    return false;
  }
  mqttConnected=true;
  mqttConnecting=false;
  mqttSubscribed=false;
  traceModemRX("[SERVEUR CMD] MQTT: CONNECT OK");

  // 5) SUBSCRIBE uniquement sur /CMD.
  String topic=mqttCmdTopic();
  traceModemTX("[SERVEUR CMD] MQTT: SUBSCRIBE "+topic);
  if(!mqttInput("AT+CMQTTSUBTOPIC="+String(MQTT_CLIENT_INDEX)+","+String(topic.length())+",1",topic,10000UL)) {
    traceModemRX("[SERVEUR CMD] MQTT: SUBTOPIC ECHEC");
    mqttConnected=false;
    return false;
  }
  if(!mqttAT("AT+CMQTTSUB="+String(MQTT_CLIENT_INDEX),"+CMQTTSUB: "+String(MQTT_CLIENT_INDEX)+",0",20000UL)) {
    traceModemRX("[SERVEUR CMD] MQTT: SUBSCRIBE ECHEC");
    mqttConnected=false;
    mqttSubscribed=false;
    return false;
  }
  mqttSubscribed=true;
  traceModemRX("[SERVEUR CMD] MQTT: SUBSCRIBE OK");
  traceModemRX("[SERVEUR CMD] MQTT: CMD EN ATTENTE SUR "+topic);
  return true;
}

bool mqttPublish(const String &topic, const String &payload, uint8_t qos, bool retain) {
  if(!mqttConnected || topic.length()==0) return false;
  traceModemTX("[SERVEUR CMD] MQTT PUBLISH TOPIC: "+topic);
  traceModemTX("[SERVEUR CMD] MQTT PUBLISH PAYLOAD: "+payload);
  if(topic.length()>500 || payload.length()>10240) return false;
  if(modemTcpBusy || modemExpectSendPrompt || smsTxBusy) return false;

  if(!mqttInput("AT+CMQTTTOPIC="+String(MQTT_CLIENT_INDEX)+","+String(topic.length()),topic,10000UL)) return false;
  if(!mqttInput("AT+CMQTTPAYLOAD="+String(MQTT_CLIENT_INDEX)+","+String(payload.length()),payload,10000UL)) return false;
  String pub="AT+CMQTTPUB="+String(MQTT_CLIENT_INDEX)+","+String(qos)+",60,"+(retain?"1":"0");
  if(!mqttAT(pub,"+CMQTTPUB: "+String(MQTT_CLIENT_INDEX)+",0",70000UL)) {
    traceModemRX("[SERVEUR CMD] MQTT PUBLISH ECHEC");
    return false;
  }
  traceModemRX("[SERVEUR CMD] MQTT PUBLISH OK");
  return true;
}

bool isValidMQTTCommand(const String &payload) {
  String p=payload;
  p.trim();
  if(!p.length()) return false;
  // Les reponses sont souvent multilignes : elles ne doivent jamais etre
  // interpretees comme une nouvelle commande apres reception du message retenu.
  if(p.indexOf('\n') >= 0 || p.indexOf('\r') >= 0) return false;

  String u=p;
  u.toUpperCase();

  // Le message de confirmation retenu n'est jamais une commande.
  if(u == "CMD EXECUTE") return false;

  // Commandes simples autorisees.
  if(u=="HELP" || u=="AIDE" || u=="COMMANDES" ||
     u=="STATUS" || u=="INFO" || u=="LOCATION" || u=="GPS" ||
     u=="GPS ON" || u=="GPS OFF" || u=="GPS STATUS" || u=="GPS?" ||
     u=="MODEM ON" || u=="MODEM OFF" || u=="MODEM STATUS" || u=="MODEM?" ||
     u=="SMS ON" || u=="SMS OFF" || u=="SMS STATUS" || u=="SMS?" ||
     u=="SERVER1 ON" || u=="SERVER1 OFF" || u=="SERVER1 STATUS" ||
     u=="SERVER2 ON" || u=="SERVER2 OFF" || u=="SERVER2 STATUS" ||
     u=="MQTT ON" || u=="MQTT OFF" || u=="MQTT STATUS" || u=="MQTT?" ||
     u=="BATTERY" || u=="BAT" ||
     u=="WIFI ON" || u=="WIFI OFF" || u=="WIFI STATUS" || u=="WIFI?" ||
     u=="LED ON" || u=="LED OFF" || u=="LED STATUS" || u=="LED?" ||
     u=="MODE NORMAL" || u=="MODE ECO" || u=="MODE PERFORMANCE" ||
     u=="MODE ULTRA ECO" || u=="MODE TEST" || u=="MODE PERMANENCE" || u=="MODE?" ||
     u=="BUFFER ON" || u=="BUFFER OFF" || u=="BUFFER STATUS" || u=="BUFFER?" ||
     u=="GPSINT?" || u=="SENDINT?" ||
     u=="STOPINT?" || u=="LONGINT?" || u=="HEARTBEAT?" ||
     u=="APN?" || u=="SERVER1?" || u=="SERVER2?" || u=="CONFIG?" ||
     u=="SAVE" || u=="RESTART" || u=="ADMINS?" || u=="MQTTREPLY?" ||
     u=="FACTORY" || u=="FACTORYRESET" || u=="RESETFACTORY") return true;

  // Commandes avec parametre : meme syntaxe que le moteur SMS.
  if(u.startsWith("GPSINT ") || u.startsWith("SENDINT ") ||
     u.startsWith("STOPINT ") ||
     u.startsWith("LONGINT ") || u.startsWith("HEARTBEAT ") ||
     u.startsWith("APN ") || u.startsWith("SERVER1 ") ||
     u.startsWith("SERVER2 ") || u.startsWith("PASS ") ||
     u.startsWith("ADMIN1 ") || u.startsWith("ADMIN2 ") || u.startsWith("ADMIN3 ") ||
     u.startsWith("MQTTREPLY ")) return true;

  // Mise a jour : le serveur fournit uniquement la version/nom du firmware.
  if(u.startsWith("UPDATE_")) {
    String rest=p.substring(7);
    rest.trim();
    int sep=rest.indexOf('_');
    if(sep<0) return false;
    String version=rest.substring(sep+1);
    version.trim();
    return buildGitHubFirmwareURL(version).length() > 0;
  }

  // Toute autre demande est ignoree sans reponse.
  return false;
}

void mqttHandleCommand(const String &payload) {
  String p=payload;
  p.trim();
  if(!p.length()) return;

  // Les commandes MQTT sont recues sur /CMD et utilisent exactement
  // le meme moteur que les commandes SMS. La reponse de la commande
  // est envoyee par SMS au premier numero administrateur configure.
  if(!isValidMQTTCommand(p)) {
    traceModemRX("[SERVEUR CMD] MQTT CMD IGNORE: "+p);
    return;
  }

String previousSender = smsSender;
String feedbackNumber = mqttCommandFeedbackNumber();

// MQTT ne depend pas de la configuration d'un numero SMS.
smsSender = feedbackNumber;

  mqttLastRx=millis();
  mqttLastCommand=p;
  mqttCommandContext = true;
  remoteCommandContext = true;
  mqttCommandResponse = "";

  traceModemRX("[SERVEUR CMD] MQTT CMD EXECUTION: "+p);

  String up=p;
  up.toUpperCase();

  // UPDATE conserve strictement sa syntaxe MQTT/SMS :
  // UPDATE_motdepasse_VERSION
  if(up.startsWith("UPDATE_")) {
    handleUpdateSMSCommand(p);
  } else {
    // Le serveur MQTT est deja authentifie par la connexion MQTT.
    // Le moteur SMS exige toutefois son mot de passe : on l'ajoute
    // automatiquement afin de reutiliser exactement les memes commandes.
    if(cfg.smsPassword.length()) {
      handleSMSCommand(p+"_"+cfg.smsPassword);
    } else {
      traceModemRX("[SERVEUR CMD] MQTT CMD REFUSEE: mot de passe SMS absent");
    }
  }

  mqttCommandContext = false;
  remoteCommandContext = false;
  smsSender = previousSender;
  mqttCommandResponse = "";
}

void taskMQTT() {
  // Le service MQTT reste actif tant que le modem est actif.
  // Les commandes restent disponibles en permanence tant que le modem est actif.
  if(!cfg.mqttEnable || !modemPower) {
    if(!modemPower) mqttResetState();
    return;
  }
  if(!modemDataReady) return;
  if(!cfg.mqttServer.length()) return;

  if(mqttConnected && mqttSubscribed) return;
  if(millis()-mqttLastAttempt < 15000UL) return;
  if(modemTcpBusy || modemExpectSendPrompt || smsTxBusy || modemWaitingResponse) return;

  mqttLastAttempt=millis();
  mqttConnecting=true;
  if(!mqttConnectAndSubscribe()) {
    mqttConnecting=false;
    mqttConnected=false;
    mqttSubscribed=false;
  }
}

void sendPendingTCPData() {
  if (!modemExpectSendPrompt || modemTcpPendingSocket > 2 ||
      modemTcpPending.length() == 0) return;

  traceModemTX("[TCP S" + String(modemTcpPendingSocket) + "] " + modemTcpPending);
  Serial.print(modemTcpPending);
  Serial.write((uint8_t)0x1A);
  Serial.flush();

  modemExpectSendPrompt = false;
  modemTcpBusy = true;
  modemTcpDataSentAt = millis();
}

void modemResetRuntimeState() {
  modemStage = MODEM_STAGE_BOOT;
  modemStageSince = millis();
  modemLastCommand = "";
  modemWaitingResponse = false;
  modemBasicConfigSent = false;
  modemAPNConfigured = false;
  modemTCPRxConfigured = false;
  modemIMEIRequested = false;
  modemATReady = false;
  modemDataReady = false;
  modemFailureCount = 0;
  modemRxExpected = 0;
  modemRxPayload = "";
  modemLine = "";
  networkRegistered = false;
  ceregStatusKnown = false;
  packetServiceAttached = false;
  tcpServiceOpen = false;
  tcpNetOpening = false;
  modemState = "BOOT";
}

void modemRequestNextCheck() {
  if (!modemPower || modemTcpBusy || modemExpectSendPrompt) return;
  unsigned long now = millis();
  if (now - modemLastCommandAt < MODEM_CMD_GAP_MS) return;
  if (modemWaitingResponse && now < modemLastCommandTimeout) return;
  if (modemWaitingResponse && now >= modemLastCommandTimeout) {
    // Un NETOPEN sans réponse ne doit pas bloquer les tentatives suivantes.
    if (modemLastCommand == "AT+NETOPEN") {
      tcpNetOpening = false;
      tcpServiceOpen = false;
      modemDataReady = false;
      modemStage = MODEM_STAGE_DATA;
    }
    modemWaitingResponse = false;
    modemFailureCount++;
  }

  switch (modemStage) {
    case MODEM_STAGE_BOOT:
      modemState = "BOOTING";
      if (now - modemStageSince >= MODEM_BOOT_WAIT_MS) {
        modemStage = MODEM_STAGE_SYNC;
        modemStageSince = now;
      }
      break;

    case MODEM_STAGE_SYNC:
      modemState = "SYNC";
      modemSendAT("AT");
      modemStage = MODEM_STAGE_SIM;
      break;

    case MODEM_STAGE_SIM:
      modemState = "SIM CHECK";
      modemSendAT("AT+CPIN?");
      modemStage = MODEM_STAGE_SIGNAL;
      break;

    case MODEM_STAGE_SIGNAL:
      modemState = "SIGNAL CHECK";
      modemSendAT("AT+CSQ");
      modemStage = MODEM_STAGE_REGISTRATION;
      break;

     //modemSendAT("AT+FSMEM");//********************************
     

    case MODEM_STAGE_REGISTRATION:
      modemState = "NETWORK SEARCH";
      // A7670E LTE : interroger l enregistrement reseau, pas le systeme fichiers.
      modemSendAT("AT+CEREG?");
      modemStage = MODEM_STAGE_ATTACH;
      break;

    case MODEM_STAGE_ATTACH:
      // Simplification : CGATT n'est plus un prérequis de connexion.
      // La présence SIM + l'enregistrement réseau autorisent les tentatives.
      modemStage = MODEM_STAGE_DATA;
      break;

    case MODEM_STAGE_DATA:
      if (!simReady) { modemStage = MODEM_STAGE_SIM; break; }
      if (!networkRegistered) { modemStage = MODEM_STAGE_REGISTRATION; break; }
      if (cfg.apn.length() && !modemAPNConfigured) {
        modemState = "CONFIG APN";
        modemSendAT("AT+CGDCONT=1,\"IP\",\"" + cfg.apn + "\"");
        modemAPNConfigured = true;
        break;
      }

      // Réception TCP A76XX en multi-connexion : demander explicitement
      // l'en-tête +RECEIVE,<link>,<length>. Sans Header-Type=1, le modem
      // peut utiliser +IPD et le numéro du serveur source (0/1) est perdu.
      // Cette configuration est appliquée avant NETOPEN, comme recommandé
      // par la documentation A76XX.
      if (!modemTCPRxConfigured) {
        modemState = "CONFIG TCP RX";
        modemSendAT("AT+CIPCCFG=10,0,0,1,1,0,500,0");
        modemTCPRxConfigured = true;
        break;
      }

      if (!tcpServiceOpen) {
        modemState = "OPEN DATA";
        if (!tcpNetOpening &&
            (lastNetOpenAttempt == 0 || now - lastNetOpenAttempt >= NET_REOPEN_INTERVAL_MS)) {
          lastNetOpenAttempt = now;
          tcpNetOpening = true;
          modemSendAT("AT+NETOPEN");
        }
        break;
      }
      modemDataReady = true;
      modemStage = MODEM_STAGE_READY;
      modemStageSince = now;
      modemState = "DATA READY";
      break;

    case MODEM_STAGE_READY:
      modemState = "ONLINE";
      // Supervision légère et périodique; les URC restent traités en continu.
      if (now - lastNetworkPoll >= 10000UL) {
        lastNetworkPoll = now;
        modemSendAT("AT+CEREG?");
      }
      break;

    case MODEM_STAGE_RECOVERY:
      modemState = "RECOVERY";
      if (now - modemRecoverySince >= MODEM_RECOVERY_DELAY_MS) {
        modemStage = MODEM_STAGE_BOOT;
        modemStageSince = now;
        modemDataReady = false;
      }
      break;
  }
}

// Déclaration anticipée : utilisée par taskModem() avant sa définition.
void handleServerReply(const String &line);

void taskModem() {
  if (simResetInProgress) return;
  if (modemOTAActive) return;
  if (!modemPower) return;
  if (cfg.energyMode == 5 && permanenceModemSleep) {
    // Sécurité : PERMANENCE V2.14 ne dort jamais. Réinitialiser l'ancien état.
    permanenceModemSleep = false;
  }

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (smsTxBusy && smsTxStage == 3 && c == '>') {
      smsTxPrompt();
      continue;
    }

    if (modemExpectSendPrompt && c == '>') {
      sendPendingTCPData();
      continue;
    }

    // Payload TCP : le modem a déjà annoncé exactement sa longueur dans
    // +RECEIVE,<link>,<length>. On lit les octets bruts jusqu'à cette longueur.
    // C'est important : la commande peut être reçue sans CR/LF final.
    if (modemRxExpected > 0) {
      modemRxPayload += c;
      if (modemRxPayload.length() >= modemRxExpected) {
        String payload = modemRxPayload.substring(0, modemRxExpected);
        modemRxPayload = modemRxPayload.substring(modemRxExpected);
        modemRxExpected = 0;

        // Nettoyage uniquement des séparateurs de ligne ajoutés par certains
        // serveurs, sans dépendre d'un "\n" pour détecter la fin TCP.
        payload.trim();
        if (payload.length()) {
          const uint8_t sourceSocket = modemRxSocket;
          traceModemRX("[TCP S" + String(sourceSocket + 1) + "] " + payload);
          String up = payload;
          up.toUpperCase();
          if (isServerCommandPayload(payload)) {
            addLog(String("TCP CMD SRV") + String(sourceSocket < 2 ? sourceSocket + 1 : 0) + ": " + up);
            processServerCommandPayload(payload);
          } else {
            handleServerReply(payload);
          }
        }
      }
      continue;
    }

    if (c == '\n') {
      modemLine.trim();
      if (modemLine.length()) {
        traceModemRX(modemLine);
        modemWaitingResponse = false;
        parseNetworkLine(modemLine);
        processModemLine(modemLine);
      }
      modemLine = "";
    } else if (c != '\r') {
      if (modemLine.length() < 320) modemLine += c;
      else modemLine = "";
    }
  }

  // Configuration de base une seule fois après synchronisation.
  // Une seule séquence : ATE0 puis AT&W. Aucun historique n'est conservé.
  if (modemStage == MODEM_STAGE_SIM && !modemBasicConfigSent && modemATReady &&
      !modemWaitingResponse && !modemTcpBusy && !modemExpectSendPrompt) {
    Serial.print("ATE0\rAT&W\r");
    modemLastActivity = millis();
    modemLastCommand = "AT&W";
    modemWaitingResponse = false;
    modemBasicConfigSent = true;
  }

  // Acquisition IMEI non bloquante, uniquement lorsque le modem est déjà allumé.
  if (modemPower && modemStage == MODEM_STAGE_SIM && !isValidIMEI(cfg.imei) &&
      !modemIMEIRequested && !modemWaitingResponse && !modemTcpBusy && millis() - modemLastCommandAt > 3000UL) {
    modemIMEIRequested = true;
    modemSendAT("AT+CGSN");
  }

  taskSMS();

  if (!smsTxBusy) modemRequestNextCheck();

  // Détection d'une absence prolongée de réponse : récupération contrôlée.
  if (modemFailureCount >= MODEM_MAX_SOFT_FAILURES) {
    modemFailureCount = 0;
    modemRecoveryCount++;
    if (modemRecoveryCount <= MODEM_MAX_POWER_CYCLES) {
      addLog("A7670E -> recovery power cycle #" + String(modemRecoveryCount));
      modemStage = MODEM_STAGE_RECOVERY;
      modemRecoverySince = millis();
      setModemPower(false);
    } else {
      modemRecoveryCount = 0;
      addLog("A7670E recovery limit reached; retry later");
    }
  }
}

// =========================
// IMEI / MODEM IDENTITY
// =========================
bool isValidIMEI(const String &s) {
  if (s.length() != 15) return false;
  for (uint8_t i = 0; i < 15; i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

String extractIMEIFromLine(String line) {
  line.trim();
  if (!line.length()) return "";

  // Réponses possibles : 15 chiffres seuls, +CGSN: 123..., IMEI: 123...
  int pos = line.indexOf("+CGSN:");
  if (pos >= 0) line = line.substring(pos + 6);
  pos = line.indexOf("IMEI:");
  if (pos >= 0) line = line.substring(pos + 5);
  line.trim();

  // Cherche une séquence de 15 chiffres dans la ligne.
  for (int i = 0; i <= (int)line.length() - 15; i++) {
    String candidate = line.substring(i, i + 15);
    if (isValidIMEI(candidate)) return candidate;
  }
  return "";
}

bool requestIMEIOnce(uint32_t timeoutMs) {
  while (Serial.available()) Serial.read();

  Serial.print("AT+CGSN\r");
  uint32_t start = millis();
  String line = "";

  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') {
        String candidate = extractIMEIFromLine(line);
        if (candidate.length() == 15) {
          cfg.imei = candidate;
          return true;
        }
        line = "";
      } else if (c >= 32 && c <= 126 && line.length() < 120) {
        line += c;
      }
    }
    yield();
  }

  // Secours pour les variantes de firmware A7670E qui répondent à AT+GSN.
  while (Serial.available()) Serial.read();
  Serial.print("AT+GSN\r");
  start = millis();
  line = "";

  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') {
        String candidate = extractIMEIFromLine(line);
        if (candidate.length() == 15) {
          cfg.imei = candidate;
          return true;
        }
        line = "";
      } else if (c >= 32 && c <= 126 && line.length() < 120) {
        line += c;
      }
    }
    yield();
  }

  return false;
}

void ensureIMEI() {
  // 1) Si un IMEI valide est déjà sauvegardé, NE PAS interroger le SIMCOM.
  if (isValidIMEI(cfg.imei)) {
    addLog("IMEI from SPIFFS: " + cfg.imei);
    return;
  }

  cfg.imei = "";

  // 2) Le SIMCOM est mis ON avant la phase de lecture.
  if (!modemPower) setModemPower(true);

  // 3) Attendre 10 secondes après la mise sous tension du A7670E.
  addLog("MODEM ON - waiting 10s before IMEI read");
  uint32_t bootStart = millis();
  while (millis() - bootStart < 10000UL) {
    yield();
  }

  // 4) Pendant 30 secondes, essayer périodiquement de récupérer l'IMEI.
  addLog("IMEI acquisition window: 30s");
  uint32_t windowStart = millis();
  uint8_t attempt = 0;

  while (millis() - windowStart < 30000UL) {
    attempt++;
    addLog("IMEI read attempt #" + String(attempt));

    if (requestIMEIOnce(1800UL)) {
      saveConfig();
      addLog("IMEI FOUND: " + cfg.imei);
      return;
    }

    // Petite pause entre les tentatives, sans busy-loop.
    uint32_t waitStart = millis();
    while (millis() - waitStart < 500UL && millis() - windowStart < 30000UL) {
      yield();
    }
  }

  // 5) Pas d'IMEI : le tracker continue son démarrage normalement.
  // Les futures tâches peuvent retenter si nécessaire sans bloquer le système.
  addLog("IMEI NOT FOUND after 30s - system continues");
}

bool validNMEA6(const String &v) {
  if (v.length() < 6) return false;
  for (uint8_t i = 0; i < 6; i++) {
    if (!isDigit(v[i])) return false;
  }
  return true;
}

String gps103DateTime() {
  // RMC fournit la date au format DDMMYY.
  // La trame demandee utilise YYMMDDHHMMSS.
  if (validNMEA6(gps.rawDate) && validNMEA6(gps.rawTime)) {
    String dd = gps.rawDate.substring(0, 2);
    String mm = gps.rawDate.substring(2, 4);
    String yy = gps.rawDate.substring(4, 6);
    String hhmmss = gps.rawTime.substring(0, 6);
    return yy + mm + dd + hhmmss;
  }
  return "000000000000";
}

String gps103Utc() {
  if (validNMEA6(gps.rawTime)) return gps.rawTime.substring(0, 6) + ".000";
  return "000000.000";
}

// Validation des coordonnees NMEA transmises au serveur.
// Le format brut est conserve : DDMM.MMMM pour latitude, DDDMM.MMMM longitude.
static bool gpsNmeaCoordinateValid(const String &raw, uint8_t degreeDigits,
                                   char hemisphere, double maxDegrees) {
  if (raw.length() < (size_t)degreeDigits + 3U) return false;
  if (degreeDigits == 2) {
    if (hemisphere != 'N' && hemisphere != 'S') return false;
  } else {
    if (hemisphere != 'E' && hemisphere != 'W') return false;
  }

  bool dotSeen = false;
  for (size_t i = 0; i < raw.length(); ++i) {
    const char c = raw[i];
    if (c == '.') {
      if (dotSeen) return false;
      dotSeen = true;
    } else if (!isDigit(c)) {
      return false;
    }
  }
  if (!dotSeen) return false;

  const double nmea = raw.toDouble();
  if (!isfinite(nmea) || nmea < 0.0) return false;
  const double degrees = (long)(nmea / 100.0);
  const double minutes = nmea - degrees * 100.0;
  if (minutes < 0.0 || minutes >= 60.0) return false;
  if (degrees > maxDegrees || (degrees == maxDegrees && minutes > 0.0)) return false;
  return true;
}

// Ne rejette jamais une position valide à cause de la vitesse.
// Une vitesse invalide est ramenée à 0 uniquement dans la trame TCP.
static double gpsSpeedForFrame() {
  if (!isfinite(gps.speed) || gps.speed < 0.0 || gps.speed > 300.0) return 0.0;
  return gps.speed;
}



static bool validateGPSForTCP() {

  // Le récepteur doit annoncer un FIX valide.
  if (!gps.fix) return false;

  // Vérification des valeurs numériques.
  if (!isfinite(gps.lat) || !isfinite(gps.lon)) {
    return false;
  }

  // Vérification des limites géographiques.
  if (gps.lat < -90.0 || gps.lat > 90.0 ||
      gps.lon < -180.0 || gps.lon > 180.0) {
    return false;
  }

  // Vérification du format des coordonnées NMEA.
  if (!gpsNmeaCoordinateValid(gps.latNmea, 2, gps.latHem, 90.0) ||
      !gpsNmeaCoordinateValid(gps.lonNmea, 3, gps.lonHem, 180.0)) {
    return false;
  }

  // Aucun contrôle de distance entre deux positions.
  // Aucun contrôle de vitesse ou de déplacement.
  // Chaque position est évaluée indépendamment.

  return true;
}



String buildGPS103Frame() {    //***************
  readBattery();
  if (cfg.imei.length() != 15 || !validateGPSForTCP()) return "";

  // Format etendu :
  // imei:IMEI,tracker,DATE,,F,HEURE,A,LAT,N,LON,E,VITESSE,CAP,ALTITUDE,ACC,SIGNAL_GPRS%,BATTERIE%,,;
  // ACC derive de la vitesse validee : > 6 km/h => 1, sinon 0.
  String frame = "imei:" + cfg.imei + ",tracker," + gps103DateTime() + ",,F,";
  frame += gps103Utc() + ",A,";
  frame += gps.latNmea;
  frame += ","; frame += gps.latHem; frame += ",";
  frame += gps.lonNmea;
  frame += ","; frame += gps.lonHem; frame += ",";

  double speedKmh = gpsSpeedForFrame();
  double course = gps.course;
  double altitude = gps.altitude;
  int accState = (speedKmh > 6.0) ? 1 : 0;

  // CSQ 0..31 converti en pourcentage indicatif. 99 = inconnu, transmis comme 0.
  int gprsPercent = 0;
  if (networkRSSI >= 0 && networkRSSI <= 31) {
    gprsPercent = (int)round((networkRSSI * 100.0) / 31.0);
  }

  frame += String(speedKmh, 1) + ",";
  frame += String(course, 0) + ",";
  frame += String(altitude, 1) + ",";
  frame += String(accState) + ",";
  frame += String(gprsPercent) + ",";
  frame += String(constrain((int)batteryPercent, 0, 100)) + ",,;";

  lastPreparedGPS103 = frame;
  lastPreparedGPS103At = millis();
  return frame;
}

String buildGPS103Heartbeat() {
  if (cfg.imei.length() != 15) return "";
  return cfg.imei + ";";
}

// =========================
// FACTORY RESET
// =========================
void factoryReset() {
  addLog("FACTORY RESET");
  SPIFFS.format();
  delay(500);
  ESP.restart();
}

// =========================
// SPIFFS
// =========================
bool saveConfig() {

  File f = SPIFFS.open("/config.json", "w");

  if (!f) {
    addLog("ERROR: config save");
    return false;
  }

  DynamicJsonDocument doc(3072);

  doc["deviceID"] = cfg.deviceID;
  doc["imei"] = cfg.imei;
  doc["apn"] = cfg.apn;

  doc["server1"] = cfg.server1;
  doc["port1"] = cfg.port1;
  doc["server1Enable"] = cfg.server1Enable;

  doc["server2"] = cfg.server2;
  doc["port2"] = cfg.port2;
  doc["server2Enable"] = cfg.server2Enable;

  doc["gpsInterval"] = cfg.gpsInterval;
  doc["positionInterval"] = cfg.positionInterval;
  doc["longIntervalMin"] = cfg.longIntervalMin;
  doc["stopDelay"] = cfg.stopDelay;
  doc["turnAngle"] = cfg.turnAngle;
  doc["heartbeatInterval"] = cfg.heartbeatInterval;
  doc["gpsTimeout"] = cfg.gpsTimeout;
  doc["motionConfirmTimeout"] = cfg.motionConfirmTimeout;
  doc["vibrationWatchSeconds"] = cfg.vibrationWatchSeconds;
  doc["sendTimeout"] = cfg.sendTimeout;
  doc["minSat"] = cfg.minSat;

  doc["bufferMax"] = cfg.bufferMax;
  doc["recoveryRate"] = cfg.recoveryRate;

  doc["energyMode"] = cfg.energyMode;

  doc["vibrationEnable"] = cfg.vibrationEnable;
  doc["smsEnable"] = cfg.smsEnable;
  doc["ledEnable"] = cfg.ledEnable;
  doc["smsAdmin1"] = cfg.smsAdmin1;
  doc["smsAdmin2"] = cfg.smsAdmin2;
  doc["smsAdmin3"] = cfg.smsAdmin3;
  doc["smsPassword"] = cfg.smsPassword;
  doc["smsPrivateCode"] = cfg.smsPrivateCode;
  doc["mqttEnable"] = true;
  doc["mqttServer"] = cfg.mqttServer;
  doc["mqttPort"] = cfg.mqttPort;
  doc["mqttUser"] = cfg.mqttUser;
  doc["mqttPassword"] = cfg.mqttPassword;
  doc["mqttKeepAlive"] = cfg.mqttKeepAlive;
  doc["mqttReplyNumber"] = cfg.mqttReplyNumber;
  doc["bufferEnable"] = cfg.bufferEnable;
  doc["transmitMode"] = cfg.transmitMode;
  doc["batteryCalibration"] = cfg.batteryCalibration;
  doc["autoPowerSave"] = cfg.autoPowerSave;
  doc["batteryLowV"] = cfg.batteryLowV;
  doc["batteryCriticalV"] = cfg.batteryCriticalV;
  doc["batteryAction"] = cfg.batteryAction;
  doc["gpsIdleOffDelay"] = cfg.gpsIdleOffDelay;
  doc["modemIdleOffDelay"] = cfg.modemIdleOffDelay;
  doc["vibrationHold"] = cfg.vibrationHold;
  doc["vibrationThreshold"] = cfg.vibrationThreshold;
  doc["vibrationDebounceMs"] = cfg.vibrationDebounceMs;

  serializeJsonPretty(doc, f);
  f.close();

  addLog("Configuration saved");
  return true;
}

// =========================
// CONFIGURATION
// =========================
void loadConfig() {

  if (!SPIFFS.exists("/config.json")) {
    addLog("No configuration - defaults");
    saveConfig();
    return;
  }

  File f = SPIFFS.open("/config.json", "r");

  if (!f) {
    addLog("ERROR: config read");
    return;
  }

  DynamicJsonDocument doc(3072);

  if (deserializeJson(doc, f)) {
    addLog("ERROR: invalid config");
    f.close();
    return;
  }

  cfg.deviceID = doc["deviceID"] | "SMART-001";
  cfg.imei = doc["imei"] | "";
  cfg.apn = doc["apn"] | "INWI.ma";

  cfg.server1 = doc["server1"] | "";
  cfg.port1 = doc["port1"] | 10200;
  cfg.server1Enable = doc["server1Enable"] | true;

  cfg.server2 = doc["server2"] | "";
  cfg.port2 = doc["port2"] | 10200;
  cfg.server2Enable = doc["server2Enable"] | false;

  cfg.gpsInterval = constrain((int)(doc["gpsInterval"] | 1), 1, 60);
  cfg.positionInterval = constrain((int)(doc["positionInterval"] | 10), 1, 3600);
  cfg.longIntervalMin = constrain((uint32_t)(doc["longIntervalMin"] | 30), (uint32_t)1, (uint32_t)10080);
  cfg.stopDelay = constrain((int)(doc["stopDelay"] | 60), 5, 3600);
  cfg.turnAngle = constrain((int)(doc["turnAngle"] | 30), 5, 180);
  cfg.heartbeatInterval = constrain((int)(doc["heartbeatInterval"] | 180), 30, 3600);
  cfg.gpsTimeout = constrain((int)(doc["gpsTimeout"] | 180), 10, 3600);
  cfg.motionConfirmTimeout = constrain((int)(doc["motionConfirmTimeout"] | 120), 10, 600);
  if (doc.containsKey("vibrationWatchSeconds")) {
    cfg.vibrationWatchSeconds = constrain((int)(doc["vibrationWatchSeconds"] | 60), 1, 3600);
  } else {
    // Migration de l'ancien réglage en minutes vers les secondes.
    cfg.vibrationWatchSeconds = constrain((int)(doc["vibrationWatchMinutes"] | 1), 1, 60) * 60;
  }
  cfg.sendTimeout = constrain((int)(doc["sendTimeout"] | 120), 10, 1800);
  cfg.minSat = constrain((int)(doc["minSat"] | 4), 0, 20);

  cfg.bufferMax = constrain((int)(doc["bufferMax"] | 7000), 10, 7000);
  cfg.recoveryRate = constrain((int)(doc["recoveryRate"] | 3), 1, 20);

  cfg.energyMode = constrain((int)(doc["energyMode"] | 0), 0, 5);

  cfg.vibrationEnable = doc["vibrationEnable"] | true;
  cfg.smsEnable = doc["smsEnable"] | true;
  cfg.ledEnable = doc["ledEnable"] | true;
  cfg.smsAdmin1 = doc["smsAdmin1"] | "";
  cfg.smsAdmin2 = doc["smsAdmin2"] | "";
  cfg.smsAdmin3 = doc["smsAdmin3"] | "";
  cfg.smsPassword = doc["smsPassword"] | "123456";
  cfg.smsPrivateCode = doc["smsPrivateCode"] | "739251";
  cfg.mqttEnable = doc["mqttEnable"] | true;
  cfg.mqttServer = doc["mqttServer"] | "broker.emqx.io";
  cfg.mqttPort = constrain((int)(doc["mqttPort"] | 1883), 1, 65535);
  cfg.mqttUser = doc["mqttUser"] | "";
  cfg.mqttPassword = doc["mqttPassword"] | "";
  cfg.mqttKeepAlive = constrain((int)(doc["mqttKeepAlive"] | 60), 10, 3600);
  cfg.mqttReplyNumber = doc["mqttReplyNumber"] | "";
  cfg.bufferEnable = true; // Enregistrement permanent : le buffer est toujours actif.
  cfg.transmitMode = doc["transmitMode"] | 0;
  cfg.batteryCalibration = doc["batteryCalibration"] | BAT_DEFAULT_CALIBRATION;
  cfg.autoPowerSave = doc["autoPowerSave"] | true;
  cfg.batteryLowV = doc["batteryLowV"] | 3.60f;
  cfg.batteryCriticalV = doc["batteryCriticalV"] | 3.50f;
  cfg.batteryAction = constrain((int)(doc["batteryAction"] | 0), 0, 2);
  cfg.gpsIdleOffDelay = constrain((int)(doc["gpsIdleOffDelay"] | 60), 10, 3600);
  cfg.modemIdleOffDelay = constrain((int)(doc["modemIdleOffDelay"] | 120), 5, 600);
  cfg.vibrationHold = constrain((int)(doc["vibrationHold"] | 8), 1, 120);
  cfg.vibrationThreshold = constrain((int)(doc["vibrationThreshold"] | 5), 1, 1000);
  cfg.vibrationDebounceMs = constrain((int)(doc["vibrationDebounceMs"] | 20), 5, 500);
  if (cfg.batteryCriticalV >= cfg.batteryLowV) cfg.batteryCriticalV = cfg.batteryLowV - 0.10f;
  if (cfg.smsPassword.length() < 4) cfg.smsPassword = "123456";
  if (cfg.smsPrivateCode.length() < 4) cfg.smsPrivateCode = "739251";

  f.close();

  addLog("Configuration loaded");
}

// =========================
// POWER
// (logique active-low confirmée par le matériel :
//  LOW sur la broche = module ON, HIGH = module OFF)
// =========================
void setGPSPower(bool state) {

  if (state == gpsPower) return;
  gpsPower = state;

  digitalWrite(GPS_POWER, state ? LOW : HIGH);
  if (state) lastGpsActivityForPower = millis();

  addLog(String("GP02 POWER ") + (state ? "ON" : "OFF"));
}

void setModemPower(bool state) {
  // Pendant le cycle SIM hot-swap, empêcher les autres tâches de contredire
  // la phase demandée : phase 1 = OFF, phase 2 = ON.
  if (simResetInProgress) {
    bool powerDemandee = (simRecoveryPhase == 2);
    if (state != powerDemandee) return;
  }


  // Recovery communication : aucune tache ne peut rallumer le modem
  // pendant la courte phase de recovery deja existante. Ensuite, le prochain
  // appel normal relance toute l'initialisation et donc un nouveau check.
  if (state && !modemPower && modemStage == MODEM_STAGE_RECOVERY &&
      millis() - modemRecoverySince < MODEM_RECOVERY_DELAY_MS) return;

  if (state == modemPower) return;
  modemPower = state;

  digitalWrite(MODEM_POWER, state ? LOW : HIGH);

  if (state) {
    lastSMSInit = 0;
    modemLine = "";
    smsSender = "";
    smsWaitingBody = false;
    smsTxBusy = false; smsTxStage = 0; smsTxNumber = ""; smsTxMessage = "";
    clearSMSQueue();
    smsModemConfigured = false; smsConfigStep = 0;
    modemBootCount++;
    lastModemActivityForPower = millis();
    lastSimPoll = 0;
    modemResetRuntimeState();
  } else {
    mqttResetState();
    tcpServiceOpen = false;
    tcpNetOpening = false;
    link1.connected = link1.opening = link1.sending = link1.loginSent = false;
    link2.connected = link2.opening = link2.sending = link2.loginSent = false;
    recoveryFrame = "";
    shortPositionPending = false;
    longPositionPending = false;
    shortEventMask = 0;
    longEventMask = 0;
    modemTcpBusy = false;
    modemExpectSendPrompt = false;
    modemTcpPending = "";
    modemTcpPendingSocket = 255;
    modemTcpStartedAt = 0;
    modemTcpDataSentAt = 0;
    bufferActiveSocket = 255;
    bufferConfirmedMask = 0;
    bufferTargetMask = 0;
  }

  addLog(String("A7670E POWER ") + (state ? "ON" : "OFF"));
}

// =========================
// BATTERIE - A0
// Pont : 220K entre BAT+ et A0, 10K entre A0 et GND
// 4.20V ou plus = 100%, 3.50V = 0%
// =========================
void readBattery() {
  if (millis() - lastBatteryRead < 1000) return;
  lastBatteryRead = millis();

  uint32_t sum = 0;
  const uint8_t samples = 20;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(A0);
    delay(1);
    yield();
  }

  batteryRawADC = (uint16_t)(sum / samples);
  batteryA0Voltage = (batteryRawADC / 1023.0f) * BAT_A0_FULL_SCALE;
  const float dividerRatio = (BAT_TOP_R + BAT_BOTTOM_R) / BAT_BOTTOM_R;
  batteryVoltage = batteryA0Voltage * dividerRatio * cfg.batteryCalibration;

  if (batteryVoltage >= BAT_FULL_V) batteryPercent = 100;
  else if (batteryVoltage <= BAT_EMPTY_V) batteryPercent = 0;
  else batteryPercent = (uint8_t)roundf(((batteryVoltage - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V)) * 100.0f);
}

String batteryStatus() {
  if (batteryVoltage >= BAT_FULL_V) return "FULL";
  if (batteryVoltage <= cfg.batteryCriticalV) return "CRITICAL";
  if (batteryVoltage <= cfg.batteryLowV) return "LOW";
  return "NORMAL";
}

String batteryText() {
  return String(batteryVoltage, 2) + " V / " + String(batteryPercent) + "%";
}

bool loadBatteryAlertState() {
  if (!SPIFFS.exists("/battery.alert")) return false;
  File f = SPIFFS.open("/battery.alert", "r");
  if (!f) return false;
  String v = f.readString();
  f.close();
  return v == "1";
}

void saveBatteryAlertState(bool sent) {
  if (sent) {
    File f = SPIFFS.open("/battery.alert", "w");
    if (f) { f.print("1"); f.close(); }
  } else {
    SPIFFS.remove("/battery.alert");
  }
  batteryAlertSent = sent;
}

void taskBatteryProtection() {
  if (batteryVoltage <= 0.0f) return;

  // Réarmement uniquement lorsque la batterie repasse au-dessus du seuil.
  if (batteryVoltage > cfg.batteryCriticalV) {
    if (batteryAlertSent) saveBatteryAlertState(false);
    batteryShutdownWait = false;
    return;
  }

  if (cfg.batteryAction == 0) return;

  if (!batteryAlertSent && cfg.smsEnable && cfg.smsAdmin1.length()) {
    if (!modemPower) setModemPower(true);
    if (!smsTxBusy && smsQueueCount == 0) {
      String msg = "BATTERIE FAIBLE\nTENSION : " + String(batteryVoltage, 2) + " V";
      smsSend(cfg.smsAdmin1, msg);
      saveBatteryAlertState(true);
      batteryShutdownWait = (cfg.batteryAction == 2);
      batteryShutdownStart = millis();
    }
    return;
  }

  if (cfg.batteryAction == 2) {
    if (!batteryShutdownWait) {
      batteryShutdownWait = true;
      batteryShutdownStart = millis();
    }
    if (!smsTxBusy && smsQueueCount == 0 || millis() - batteryShutdownStart >= 30000UL) {
      batteryShutdownWait = false;
      enterConfiguredDeepSleep("BATTERIE FAIBLE");
    }
  }
}

// =========================
// GPS - BUFFER DE TRAME
// =========================
String nmeaLine = "";
// =========================
// CONVERSION COORDONNÉES NMEA -> DÉCIMAL
// =========================
double nmeaToDecimal(String raw, int degreeDigits) {

  if (raw.length() == 0) return 0;

  String degStr = raw.substring(0, degreeDigits);
  String minStr = raw.substring(degreeDigits);

  double degrees = degStr.toDouble();
  double minutes = minStr.toDouble();

  return degrees + (minutes / 60.0);
}

// =========================
// DECOUPAGE D'UNE TRAME EN CHAMPS
// =========================
int splitFields(String line, String fields[], int maxFields) {

  int count = 0;
  int start = 0;

  while (count < maxFields) {

    int idx = line.indexOf(',', start);

    if (idx == -1) {
      fields[count++] = line.substring(start);
      break;
    }

    fields[count++] = line.substring(start, idx);
    start = idx + 1;
  }

  return count;
}

// =========================
// VERIFICATION CHECKSUM
// =========================
bool checkChecksum(String line) {

  int star = line.indexOf('*');
  if (star == -1) return false;

  byte checksum = 0;
  for (int i = 1; i < star; i++)   // saute le '$'
    checksum ^= line[i];

  String csHex = line.substring(star + 1);
  csHex.trim();

  byte received = (byte) strtol(csHex.c_str(), NULL, 16);

  return checksum == received;
}

// ============================================================
// VALIDATION GPS SOUPLE ET SYNCHRONISEE GGA/RMC
// - Les deux phrases doivent concerner la meme seconde UTC.
// - Les coordonnees doivent etre numeriquement/geographiquement valides.
// - Un saut tres important est mis en attente puis confirme par la
//   position suivante, au lieu d'etre rejete sur une seule mesure.
// ============================================================
String gpsGgaEpoch = "";
String gpsRmcEpoch = "";
bool gpsGgaReady = false;
bool gpsRmcReady = false;

double gpsGgaLatCandidate = 0.0;
double gpsGgaLonCandidate = 0.0;
double gpsGgaAltitudeCandidate = 0.0;
String gpsGgaLatRawCandidate = "";
String gpsGgaLonRawCandidate = "";
char gpsGgaLatHemCandidate = 'N';
char gpsGgaLonHemCandidate = 'E';
uint8_t gpsGgaQualityCandidate = 0;
uint8_t gpsGgaSatCandidate = 0;

double gpsRmcLatCandidate = 0.0;
double gpsRmcLonCandidate = 0.0;
String gpsRmcLatRawCandidate = "";
String gpsRmcLonRawCandidate = "";
char gpsRmcLatHemCandidate = 'N';
char gpsRmcLonHemCandidate = 'E';
char gpsRmcStatusCandidate = 'V';
double gpsRmcSpeedCandidate = 0.0;
double gpsRmcCourseCandidate = 0.0;
String gpsRmcDateCandidate = "";

bool gpsJumpPending = false;
double gpsJumpLat = 0.0;
double gpsJumpLon = 0.0;
unsigned long gpsJumpAt = 0;

String gpsNmeaEpochKey(const String &rawTime) {
  if (!validNMEA6(rawTime)) return "";
  return rawTime.substring(0, 6);
}

bool gpsParseNmeaCoordinate(const String &raw, uint8_t degreeDigits,
                            char hemisphere, double maxDegrees,
                            double &decimal) {
  if (raw.length() < (size_t)degreeDigits + 3U) return false;
  if (degreeDigits == 2 && hemisphere != 'N' && hemisphere != 'S') return false;
  if (degreeDigits == 3 && hemisphere != 'E' && hemisphere != 'W') return false;

  bool dotSeen = false;
  for (size_t i = 0; i < raw.length(); ++i) {
    char c = raw[i];
    if (c == '.') {
      if (dotSeen) return false;
      dotSeen = true;
    } else if (!isDigit(c)) {
      return false;
    }
  }
  if (!dotSeen) return false;

  double nmea = raw.toDouble();
  if (!isfinite(nmea) || nmea < 0.0) return false;

  double degrees = floor(nmea / 100.0);
  double minutes = nmea - degrees * 100.0;
  if (minutes < 0.0 || minutes >= 60.0 ||
      degrees > maxDegrees ||
      (degrees == maxDegrees && minutes > 0.0)) return false;

  decimal = degrees + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
  return isfinite(decimal) &&
         decimal >= -maxDegrees && decimal <= maxDegrees;
}

double gpsDistanceMeters(double lat1, double lon1,
                          double lat2, double lon2) {
  const double rad = 0.017453292519943295;
  double dLat = (lat2 - lat1) * rad;
  double dLon = (lon2 - lon1) * rad;
  double a = sin(dLat * 0.5) * sin(dLat * 0.5) +
             cos(lat1 * rad) * cos(lat2 * rad) *
             sin(dLon * 0.5) * sin(dLon * 0.5);
  a = constrain(a, 0.0, 1.0);
  return 6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

bool gpsJumpNeedsConfirmation(double lat, double lon) {
  if (gps.lastFix == 0) return false;

  unsigned long now = millis();
  unsigned long elapsedMs = now - gps.lastFix;

  // Apres une longue perte GPS, la nouvelle zone peut etre legitime.
  if (elapsedMs > 300000UL) return false;

  double distance = gpsDistanceMeters(gps.lat, gps.lon, lat, lon);
  double elapsedSec = elapsedMs / 1000.0;

  // Seuil tolerant : 300 m de marge + environ 252 km/h.
  // Il detecte les erreurs massives sans filtrer les petits ecarts.
  double allowedDistance = 300.0 + elapsedSec * 70.0;

  if (distance <= allowedDistance) {
    gpsJumpPending = false;
    return false;
  }

  // Une deuxieme mesure proche de la premiere position suspecte
  // confirme le changement de zone.
  if (gpsJumpPending && now - gpsJumpAt <= 10000UL &&
      gpsDistanceMeters(gpsJumpLat, gpsJumpLon, lat, lon) <= 300.0) {
    gpsJumpPending = false;
    return false;
  }

  gpsJumpPending = true;
  gpsJumpLat = lat;
  gpsJumpLon = lon;
  gpsJumpAt = now;
  return true;
}

void gpsTryCommitMatchedFix() {
  if (!gpsGgaReady || !gpsRmcReady) return;
  if (gpsGgaEpoch != gpsRmcEpoch) return;

  // Une paire invalide ne doit jamais reutiliser les anciennes coordonnees.
  gps.fix = false;

  uint8_t minSat = cfg.minSat;
  if (minSat > 3) minSat--;       // marge d'un satellite, minimum 3
  if (minSat < 3) minSat = 3;

  bool validPair =
      gpsGgaQualityCandidate > 0 &&
      gpsGgaSatCandidate >= minSat &&
      gpsRmcStatusCandidate == 'A' &&
      gpsGgaLatRawCandidate.length() &&
      gpsGgaLonRawCandidate.length() &&
      gpsRmcLatRawCandidate.length() &&
      gpsRmcLonRawCandidate.length();

  // GGA et RMC doivent aussi donner des positions coherentes.
  if (validPair) {
    double pairDifference = gpsDistanceMeters(
        gpsGgaLatCandidate, gpsGgaLonCandidate,
        gpsRmcLatCandidate, gpsRmcLonCandidate);
    if (!isfinite(pairDifference) || pairDifference > 250.0)
      validPair = false;
  }

  if (validPair &&
      gpsJumpNeedsConfirmation(gpsGgaLatCandidate, gpsGgaLonCandidate)) {
    addLog("GPS : POSITION ELOIGNEE A CONFIRMER");
    validPair = false;
  }

  if (validPair) {
    gps.lat = gpsGgaLatCandidate;
    gps.lon = gpsGgaLonCandidate;
    gps.altitude = gpsGgaAltitudeCandidate;
    gps.latNmea = gpsGgaLatRawCandidate;
    gps.lonNmea = gpsGgaLonRawCandidate;
    gps.latHem = gpsGgaLatHemCandidate;
    gps.lonHem = gpsGgaLonHemCandidate;
    gps.fixQuality = gpsGgaQualityCandidate;
    gps.satellites = gpsGgaSatCandidate;
    gps.rmcStatus = gpsRmcStatusCandidate;
    gps.speed = gpsRmcSpeedCandidate;     // unite conservee : noeuds
    gps.course = gpsRmcCourseCandidate;

    if (gpsRmcDateCandidate.length() >= 6 &&
        validNMEA6(gpsRmcDateCandidate)) {
      gps.rawDate = gpsRmcDateCandidate.substring(0, 6);
      gps.date = gps.rawDate.substring(0, 2) + "/" +
                 gps.rawDate.substring(2, 4) + "/20" +
                 gps.rawDate.substring(4, 6);
    }

    gps.rawTime = gpsRmcEpoch;
    gps.utc = gpsRmcEpoch.substring(4, 6) + ":" +
              gpsRmcEpoch.substring(2, 4) + ":" +
              gpsRmcEpoch.substring(0, 2);
    gps.fix = true;
    gps.lastFix = millis();
  }

  // Eviter de recombiner une phrase ancienne avec une nouvelle seconde.
  gpsGgaReady = false;
  gpsRmcReady = false;
  gpsGgaEpoch = "";
  gpsRmcEpoch = "";
}

// =========================
// PARSING GGA (position + fix + satellites + altitude)
// Couvre $GPGGA et $GNGGA (talker ID ignoré)
// =========================
void parseGGA(String line) {
  String f[15];
  int n = splitFields(line, f, 15);
  if (n < 10) return;

  gps.fix = false;  // attendre la phrase RMC correspondante
  gpsGgaEpoch = gpsNmeaEpochKey(f[1]);
  if (!gpsGgaEpoch.length()) {
    gpsGgaReady = false;
    return;
  }

  int quality = f[6].toInt();
  int satellites = f[7].toInt();

  double lat = 0.0, lon = 0.0;
  char latHem = (f[3].length() ? f[3][0] : '?');
  char lonHem = (f[5].length() ? f[5][0] : '?');

  bool coordinatesOK =
      gpsParseNmeaCoordinate(f[2], 2, latHem, 90.0, lat) &&
      gpsParseNmeaCoordinate(f[4], 3, lonHem, 180.0, lon) &&
      !(lat == 0.0 && lon == 0.0);

  gpsGgaReady = coordinatesOK;
  gpsGgaQualityCandidate = (uint8_t)constrain(quality, 0, 9);
  gpsGgaSatCandidate = (uint8_t)constrain(satellites, 0, 99);
  gpsGgaLatCandidate = lat;
  gpsGgaLonCandidate = lon;
  gpsGgaAltitudeCandidate = f[9].length() ? f[9].toDouble() : 0.0;
  gpsGgaLatRawCandidate = f[2];
  gpsGgaLonRawCandidate = f[4];
  gpsGgaLatHemCandidate = latHem;
  gpsGgaLonHemCandidate = lonHem;

  if (!isfinite(gpsGgaAltitudeCandidate)) {
    gpsGgaAltitudeCandidate = 0.0;
  }

  gpsTryCommitMatchedFix();
}

// =========================
// PARSING RMC (vitesse + cap + date + statut)
// Couvre $GPRMC et $GNRMC
// =========================
void parseRMC(String line) {
  String f[13];
  int n = splitFields(line, f, 13);
  if (n < 10) return;

  gpsRmcEpoch = gpsNmeaEpochKey(f[1]);
  if (!gpsRmcEpoch.length()) {
    gpsRmcReady = false;
    return;
  }

  double lat = 0.0, lon = 0.0;
  char latHem = (f[4].length() ? f[4][0] : '?');
  char lonHem = (f[6].length() ? f[6][0] : '?');

  bool coordinatesOK =
      gpsParseNmeaCoordinate(f[3], 2, latHem, 90.0, lat) &&
      gpsParseNmeaCoordinate(f[5], 3, lonHem, 180.0, lon) &&
      !(lat == 0.0 && lon == 0.0);

  gpsRmcReady = coordinatesOK;
  gpsRmcStatusCandidate = (f[2].length() ? f[2].charAt(0) : 'V');
  gpsRmcLatCandidate = lat;
  gpsRmcLonCandidate = lon;
  gpsRmcLatRawCandidate = f[3];
  gpsRmcLonRawCandidate = f[5];
  gpsRmcLatHemCandidate = latHem;
  gpsRmcLonHemCandidate = lonHem;

  gpsRmcSpeedCandidate = f[7].length() ? f[7].toDouble() : 0.0;
  gpsRmcCourseCandidate = f[8].length() ? f[8].toDouble() : 0.0;
  if (!isfinite(gpsRmcSpeedCandidate) || gpsRmcSpeedCandidate < 0.0)
    gpsRmcSpeedCandidate = 0.0;
  if (!isfinite(gpsRmcCourseCandidate) ||
      gpsRmcCourseCandidate < 0.0 || gpsRmcCourseCandidate > 360.0)
    gpsRmcCourseCandidate = 0.0;

  gpsRmcDateCandidate =
      (f[9].length() >= 6 && validNMEA6(f[9]))
      ? f[9].substring(0, 6) : "";

  gpsTryCommitMatchedFix();
}

// =========================
// LECTURE + DECODAGE NMEA (SANS LIBRAIRIE)
// Seules les éléments GGA et RMC sont traitées
// (variantes GP et GN) ; tout autre type est ignoré.
// =========================
String lastNMEA = "--";
String lastNMEAType = "--";
bool lastNMEAChecksumOK = false;
unsigned long lastNMEAAt = 0;
uint32_t nmeaRxCount = 0;
uint32_t nmeaValidCount = 0;
uint32_t nmeaChecksumErrors = 0;
unsigned long lastNMEARx = 0;

void readGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      nmeaLine.trim();
      if (nmeaLine.startsWith("$") && nmeaLine.length() >= 6) {
        lastNMEA = nmeaLine;
        traceGPSTX(nmeaLine);
        lastNMEAAt = millis();
        lastNMEARx = lastNMEAAt;
        nmeaRxCount++;
        lastNMEAChecksumOK = checkChecksum(nmeaLine);
        if (lastNMEAChecksumOK) nmeaValidCount++;
        else nmeaChecksumErrors++;
        if (nmeaLine.length() >= 6) lastNMEAType = nmeaLine.substring(3, 6);
        if (lastNMEAChecksumOK) {
          String type = lastNMEAType;
          if (type == "GGA") parseGGA(nmeaLine);
          else if (type == "RMC") parseRMC(nmeaLine);
        }
      }
      nmeaLine = "";
    } else if (c != '\r') {
      if (nmeaLine.length() < 180) nmeaLine += c;
      else nmeaLine = "";
    }
  }

  if (gps.fix && (millis() - gps.lastFix > cfg.gpsTimeout * 1000UL))
    gps.fix = false;
}

// =========================
// WEB CSS - chargé une seule fois par le navigateur
// =========================
const char WEB_CSS[] PROGMEM = R"rawliteral(
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;font-family:Arial,Segoe UI,sans-serif;background:#07111f;color:#eaf2ff;font-size:14px;line-height:1.45}.sidebar{position:fixed;left:0;top:0;bottom:0;width:215px;background:#0a1627;border-right:1px solid #20344d;padding:16px 10px;z-index:5}.logo{text-align:center;padding:8px 4px 18px}.brandText{font-size:21px;font-weight:900}.brandRed{color:#ef4444}.brandWhite{color:#fff}.modelName{color:#38bdf8;font-size:11px;font-weight:800;letter-spacing:1.5px;margin-top:4px}.nav{display:flex;gap:9px;align-items:center;padding:9px 11px;margin:4px 0;color:#aebbd0;text-decoration:none;border-radius:10px;font-weight:700}.nav:hover{background:#11243b;color:#fff}.navIcon{width:22px;text-align:center;color:#38bdf8}.main{margin-left:215px;padding:22px;max-width:1450px}.top{display:flex;align-items:center;justify-content:space-between;margin-bottom:18px;padding-bottom:12px;border-bottom:1px solid #20344d}.title{font-size:24px;font-weight:800;color:#fff}.hero{display:flex;justify-content:space-between;align-items:center;gap:15px;padding:20px;margin-bottom:16px;background:linear-gradient(120deg,#132b48,#0c1b2e);border:1px solid #294663;border-radius:15px}.heroTitle{font-size:27px;font-weight:850}.heroSub,.small{color:#8fa3bb;font-size:12px}.eyebrow{color:#38bdf8;font-size:10px;font-weight:800;letter-spacing:1.3px}.heroBadge,.pill{padding:7px 11px;border-radius:999px;font-size:11px;font-weight:800;border:1px solid}.ok{color:#38bdf8;background:#0c273b;border-color:#235a78}.bad{color:#fca5a5;background:#2a1820;border-color:#63333f}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px}.card,.section{background:#0d1b2d;border:1px solid #223a55;border-radius:13px;padding:16px;box-shadow:0 7px 20px rgba(0,0,0,.16)}.card h3,.section h3{margin:0 0 8px;color:#91a4bc;font-size:11px;letter-spacing:.6px;text-transform:uppercase}.value{font-size:22px;font-weight:800;color:#38bdf8}.good{color:#38bdf8}.red{color:#fca5a5}.section{margin-top:14px}.sectionHead{display:flex;justify-content:space-between;align-items:center;gap:10px}.progress{height:9px;margin:13px 0 7px;background:#07111f;border:1px solid #20344d;border-radius:9px;overflow:hidden}.progress i{display:block;height:100%;background:#38bdf8}.progressText{display:flex;justify-content:space-between;color:#8fa3bb;font-size:11px}.statsRow{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;margin-top:12px}.statsRow div{background:#091626;border:1px solid #1d334c;border-radius:9px;padding:10px}.statsRow b,.statsRow span{display:block}.statsRow b{font-size:10px;color:#8fa3bb;text-transform:uppercase}.statsRow span{font-size:17px;font-weight:800;color:#38bdf8;margin-top:3px}.row{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px}label{display:block;color:#aebbd0;font-size:12px;font-weight:700;margin-top:3px}input,select{width:100%;padding:9px 10px;margin:5px 0 12px;background:#081522;color:#edf5ff;border:1px solid #29405b;border-radius:8px;font:inherit}input:focus,select:focus{border-color:#38bdf8;outline:none}input[type=checkbox]{width:auto;margin-right:6px}button{border:0;border-radius:8px;padding:9px 14px;cursor:pointer;font-weight:800;background:#168cff;color:#fff;margin:3px}button:hover{filter:brightness(1.08)}hr{border:0;border-top:1px solid #20344d;margin:17px 0}.actions{display:flex;flex-wrap:wrap;gap:5px;margin-top:14px}.actions a{text-decoration:none}.mono{font-family:monospace;font-size:15px;word-break:break-all}.muted{color:#72869e}@media(max-width:700px){.sidebar{position:relative;width:100%;height:auto;display:flex;flex-wrap:wrap;padding:8px}.logo{width:100%;padding:4px 4px 8px}.nav{padding:7px 8px;margin:2px;font-size:11px}.main{margin:0;padding:12px}.title{font-size:20px}.hero{padding:15px}.heroTitle{font-size:22px}.grid{grid-template-columns:repeat(auto-fit,minmax(145px,1fr))}.card,.section{padding:13px}}
)rawliteral";

String htmlStart(String title) {
  server.sendHeader("Cache-Control","no-store,no-cache,must-revalidate,max-age=0");
  String h;
  h.reserve(1200);
  h = F("<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>NUMOTRONIC ");
  h += NUMO_VERSION;
  h += F("</title><link rel='stylesheet' href='/style.css'></head><body><aside class='sidebar'><div class='logo'><div class='brandText'><span class='brandRed'>NUMO</span><span class='brandWhite'>TRONIC</span></div><div class='modelName'>");
  h += NUMO_VERSION;
  h += F("</div></div><a class='nav' href='/info'><span class='navIcon'>⌂</span>INFO</a><a class='nav' href='/config'><span class='navIcon'>⚙</span>CONFIGURATION</a><a class='nav' href='/gps'><span class='navIcon'>◉</span>GPS</a><a class='nav' href='/network'><span class='navIcon'>◈</span>RÉSEAU</a><a class='nav' href='/buffer'><span class='navIcon'>▤</span>MÉMOIRE</a><a class='nav' href='/update'><span class='navIcon'>↻</span>MISE À JOUR</a></aside><main class='main'><div class='top'><div class='title'>");
  h += title;
  h += F("</div></div>");
  return h;
}
String htmlEnd(){return F("</main></body></html>");}
void webBegin(const String &title){server.setContentLength(CONTENT_LENGTH_UNKNOWN);server.send(200,"text/html; charset=utf-8",htmlStart(title));}
void webEnd(){server.sendContent(htmlEnd());}

void pageInfo(){
  webBegin("Tableau de bord");
  const bool srv=link1.connected||link2.connected;
  const uint32_t cap=cfg.bufferMax?cfg.bufferMax:1;
  const uint32_t pct=min<uint32_t>(100,(bufferPending*100UL)/cap);
  const uint32_t txAge=lastGlobalTcpTx?(millis()-lastGlobalTcpTx)/1000UL:0;
  String c;c.reserve(4200);
  c+=F("<div class='hero'><div><div class='eyebrow'>NUMOTRONIC • ÉTAT GÉNÉRAL</div><div class='heroTitle'>NUMO TRACKER</div><div class='heroSub'>NOUVEAU EDITION.</div></div><div class='heroBadge ");c+=srv?F("ok'>● CONNECTÉ"):F("bad'>● DÉCONNECTÉ");c+=F("</div></div><div class='grid'>");
  c+=F("<div class='card'><h3>VERSION / IMEI</h3><div class='value'>");c+=NUMO_VERSION;c+=F("</div><div class='small mono'>");c+=cfg.imei.length()?cfg.imei:F("—");c+=F("</div></div>");
  c+=F("<div class='card'><h3>MODE</h3><div class='value'>");
  switch(cfg.energyMode){case 0:c+=F("NORMAL");break;case 1:c+=F("ECO");break;case 2:c+=F("PERFORMANCE");break;case 3:c+=F("ULTRA ECO");break;default:c+=F("TEST");break;}c+=F("</div><div class='small'>Mode de fonctionnement</div></div>");
  c+=F("<div class='card'><h3>GPS</h3><div class='value ");c+=gps.fix?F("good'>FIX"):F("red'>NO FIX");c+=F("</div><div class='small'>");c+=gpsPower?F("Actif"):F("En veille");c+=F("</div></div>");
  c+=F("<div class='card'><h3>SERVEUR</h3><div class='value ");c+=srv?F("good'>CONNECTÉ"):F("red'>DÉCONNECTÉ");c+=F("</div><div class='small'>S1 ");c+=link1.connected?F("OK"):F("—");c+=F(" · S2 ");c+=link2.connected?F("OK"):F("—");c+=F("</div></div>");
  c+=F("<div class='card'><h3>BATTERIE</h3><div class='value'>");c+=String(batteryPercent);c+=F(" %</div><div class='small'>");c+=String (batteryVoltage,2) ;c+=F(" V</div></div>");
  c+=F("<div class='card'><h3>TRAMES EN ATTENTE</h3><div class='value'>");c+=String(bufferPending);c+=F("</div><div class='small'>sur ");c+=String(cfg.bufferMax);c+=F("</div></div>");
  c+=F("<div class='card'><h3>TRAMES ENVOYÉES</h3><div class='value'>");c+=String(bufferSent);c+=F("</div><div class='small'>confirmées</div></div>");
  c+=F("<div class='card'><h3>ÉCHECS</h3><div class='value ");c+=tcpTxFailure?F("red'>"):F("good'>");c+=String(tcpTxFailure);c+=F("</div><div class='small'>transmissions non confirmées</div></div></div>");
  c+=F("<div class='section'><div class='sectionHead'><h3>TRANSMISSION / MÉMOIRE</h3><span class='pill ");c+=srv?F("ok'>LIEN ACTIF"):F("bad'>EN ATTENTE");c+=F("</span></div><div class='progress'><i style='width:");c+=String(pct);c+=F("%'></i></div><div class='progressText'><span>");c+=String(bufferPending);c+=F(" / ");c+=String(cfg.bufferMax);c+=F(" trames</span><span>");c+=String(pct);c+=F(" %</span></div><div class='statsRow'><div><b>Envoyées</b><span>");c+=String(bufferSent);c+=F("</span></div><div><b>Perdues</b><span>");c+=String(bufferLost);c+=F("</span></div><div><b>Tentatives</b><span>");c+=String(bufferRetry);c+=F("</span></div><div><b>Dernier envoi</b><span>");c+=lastGlobalTcpTx?String(txAge)+" s":String("—");c+=F("</span></div></div></div>");
  c+=F("<div class='grid'><div class='card'><h3>POSITION</h3><div class='value'>");if(gps.fix){c+=String(gps.lat,6);c+=F("<br>");c+=String(gps.lon,6);}else c+=F("—");c+=F("</div><div class='small'>");c+=String(gps.speed,1);c+=F(" km/h · ");c+=String(gps.satellites);c+=F(" satellites</div></div><div class='card'><h3>APPAREIL</h3><div class='value'>ACTIF</div><div class='small'>fonctionnement depuis ");c+=String((millis()-bootTime)/3600000UL);c+=F(" h</div></div></div><div class='actions'><a href='/gps'><button>GPS</button></a><a href='/network'><button>RÉSEAU</button></a><a href='/buffer'><button>MÉMOIRE</button></a><a href='/update'><button>MISE À JOUR</button></a></div>");
  server.sendContent(c);webEnd();
}

void pageConfig(){
  webBegin("Configuration");
  String c;c.reserve(7200);
  c+=F("<form action='/save' method='POST'><div class='section'><h3>IDENTIFICATION</h3><label>Identifiant appareil</label><input name='deviceID' value='");c+=cfg.deviceID;c+=F("'></div><div class='section'><h3>LOCALISATION / INTERVALLES</h3><div class='row'>");
  c+=F("<div><label>Lecture GPS (s)</label><input type='number' name='gpsInterval' value='");c+=String(cfg.gpsInterval);c+=F("'></div><div><label>Enregistrement position (s)</label><input type='number' name='positionInterval' value='");c+=String(cfg.positionInterval);c+=F("'></div><div><label>REVEIL. (min)</label><input type='number' name='longIntervalMin' value='");c+=String(cfg.longIntervalMin);c+=F("'></div><div><label>Arrêt complet (s)</label><input type='number' name='stopDelay' value='");c+=String(cfg.stopDelay);c+=F("'></div><div><label>Angle cap (°)</label><input type='number' name='turnAngle' value='");c+=String(cfg.turnAngle);c+=F("'></div><div><label>Maintien communication (s)</label><input type='number' name='heartbeatInterval' value='");c+=String(cfg.heartbeatInterval);c+=F("'></div><div><label>Timeout GPS (s)</label><input type='number' name='gpsTimeout' value='");c+=String(cfg.gpsTimeout);c+=F("'></div>");c+=F("<div><label>Recherche FIX après vibration (s)</label><input type='number' name='motionConfirmTimeout' min='10' max='600' value='");c+=String(cfg.motionConfirmTimeout);c+=F("'></div><div><label>Surveillance vibration (s)</label><input type='number' name='vibrationWatchSeconds' min='1' max='3600' value='");c+=String(cfg.vibrationWatchSeconds);c+=F("'></div><div><label>Timeout d'envoi (s)</label><input type='number' name='sendTimeout' min='10' max='1800' value='");c+=String(cfg.sendTimeout);c+=F("'></div><div><label>Satellites minimum</label><input type='number' name='minSat' value='");c+=String(cfg.minSat);c+=F("'></div></div></div>");
  c+=F("<div class='section'><h3>RÉSEAU / SERVEURS</h3><label>Accès Internet mobile</label><input name='apn' value='");c+=cfg.apn;c+=F("'>");
  c+=F("<div class='row'><div><label>Serveur 1</label><input name='server1' autocomplete='off' value='");c+=cfg.server1;c+=F("'></div><div><label>Port de transmission 1</label><input type='number' name='port1' min='1' max='65535' value='");c+=String(cfg.port1);c+=F("'></div></div>");
  c+=F("<label><input type='checkbox' name='server1Enable' ");if(cfg.server1Enable)c+=F("checked");c+=F(">Activer serveur 1</label>");
  c+=F("<div class='section' style='margin-top:12px'><div class='sectionHead'><h3>SERVEUR 2 — CONFIGURATION</h3><span class='pill ");c+=cfg.server2Enable?F("ok'>ACTIF"):F("bad'>INACTIF");c+=F("</span></div><div class='row'><div><label>Adresse du serveur 2</label><input name='server2' autocomplete='off' value='");c+=cfg.server2;c+=F("'></div><div><label>Port de transmission 2</label><input type='number' name='port2' min='1' max='65535' value='");c+=String(cfg.port2);c+=F("'></div></div><label><input type='checkbox' name='server2Enable' ");if(cfg.server2Enable)c+=F("checked");c+=F(">Activer serveur 2</label></div></div>");
  c+=F("<div class='section'><h3>ÉNERGIE / RÉVEIL</h3><div class='row'><div><label>Mode</label><select name='energyMode'>");
  // Ordre d'affichage demandé. Les valeurs internes restent inchangées
  // pour préserver la compatibilité avec les configurations déjà enregistrées.
  const char* modes[]={"NORMAL","ECO","ULTRA ECO","PERFORMANCE","PERMANENCE","TEST"};
  const uint8_t modeValues[]={0,1,3,2,5,4};
  for(uint8_t i=0;i<6;i++){uint8_t modeValue=modeValues[i];c+=F("<option value='");c+=String(modeValue);c+=F("' ");if(cfg.energyMode==modeValue)c+=F("selected");c+=F(">");c+=modes[i];c+=F("</option>");}
  c+=F("</select></div><div><label>Gestion batterie</label><select name='batteryAction'>"); c+=F("<option value='0' "); if(cfg.batteryAction==0)c+=F("selected"); c+=F(">BIEN</option><option value='1' "); if(cfg.batteryAction==1)c+=F("selected"); c+=F(">AVERTIR</option><option value='2' "); if(cfg.batteryAction==2)c+=F("selected"); c+=F(">AVERTIR ET ARRÊTER</option>"); c+=F("</select></div><div><label>Sensibilité de vibration</label><select name='vibrationSensitivity'>");
  c+=F("<option value='LOW' ");if(cfg.vibrationThreshold<=3)c+=F("selected");c+=F(">FAIBLE</option><option value='MEDIUM' ");if(cfg.vibrationThreshold>3 && cfg.vibrationThreshold<=7)c+=F("selected");c+=F(">MOYENNE</option><option value='HIGH' ");if(cfg.vibrationThreshold>7)c+=F("selected");c+=F(">ÉLEVÉE</option>");
  c+=F("</select></div></div><div class='small'>Le REVEIL est un intervalle général configuré dans la rubrique LOCALISATION / INTERVALLES.</div></div>");
  c+=F("<div class='section'><h3>SMS / OPTIONS</h3><div class='row'><div><label>Numéro autorisé 1</label><input name='smsAdmin1' value='");c+=cfg.smsAdmin1;c+=F("'></div><div><label>Numéro autorisé 2</label><input name='smsAdmin2' value='");c+=cfg.smsAdmin2;c+=F("'></div><div><label>Numéro autorisé 3</label><input name='smsAdmin3' value='");c+=cfg.smsAdmin3;c+=F("'></div><div><label>Mot de passe SMS</label><input name='smsPassword' type='text' value='");c+=cfg.smsPassword;c+=F("'></div></div><div class='small'>Les accès privés sont disponibles uniquement dans la page DIAGNOSTIC.</div><label><input type='checkbox' name='vibrationEnable' ");if(cfg.vibrationEnable)c+=F("checked");c+=F(">Détection vibration</label><label><input type='checkbox' name='smsEnable' ");if(cfg.smsEnable)c+=F("checked");c+=F(">Commandes SMS</label><label><input type='checkbox' name='ledEnable' ");if(cfg.ledEnable)c+=F("checked");c+=F(">Voyant actif</label></div>");
c+=F("<div class='section'><h3>MAINTENANCE</h3><div class='actions'><button type='button' onclick='if(confirm(\"Redémarrer l’appareil ?\")) location.href=\"/restart\";'>↻ REDÉMARRER</button><button type='button' onclick='if(confirm(\"Restaurer les valeurs usine ? Toutes les configurations personnalisées seront remplacées.\")) location.href=\"/factory-reset\";'>⚙ VALEURS USINE</button></div><div class='small'>Ces actions n’affectent pas les autres réglages.</div></div>");
c+=F("<div class='section'><button type='submit'>ENREGISTRER LA CONFIGURATION</button></div></form>");
  server.sendContent(c);webEnd();
}

// =========================
// SAVE (révisée : prise en compte des serveurs)
// =========================
void saveFromWeb() {

  const String oldServer2 = cfg.server2;
  const uint16_t oldPort2 = cfg.port2;
  const bool oldServer2Enable = cfg.server2Enable;

  if (server.hasArg("deviceID"))
    cfg.deviceID = server.arg("deviceID");

  if (server.hasArg("apn"))
    cfg.apn = server.arg("apn");

  if (server.hasArg("server1"))
    cfg.server1 = server.arg("server1");

  if (server.hasArg("port1"))
    cfg.port1 = server.arg("port1").toInt();

  cfg.server1Enable = server.hasArg("server1Enable");

  if (server.hasArg("server2"))
    cfg.server2 = server.arg("server2");

  if (server.hasArg("port2"))
    cfg.port2 = server.arg("port2").toInt();

  cfg.server2Enable = server.hasArg("server2Enable");

  cfg.gpsInterval = constrain(server.arg("gpsInterval").toInt(), 1, 60);
  cfg.positionInterval = constrain(server.arg("positionInterval").toInt(), 1, 3600);
  cfg.longIntervalMin = constrain((uint32_t)server.arg("longIntervalMin").toInt(), (uint32_t)1, (uint32_t)10080);
  cfg.stopDelay = constrain(server.arg("stopDelay").toInt(), 5, 3600);
  cfg.turnAngle = constrain(server.arg("turnAngle").toInt(), 5, 180);
  cfg.heartbeatInterval = constrain(server.arg("heartbeatInterval").toInt(), 30, 3600);
  cfg.gpsTimeout = constrain(server.arg("gpsTimeout").toInt(), 10, 3600);
  cfg.motionConfirmTimeout = constrain(server.arg("motionConfirmTimeout").toInt(), 10, 600);
  if (server.hasArg("vibrationWatchSeconds")) cfg.vibrationWatchSeconds = constrain(server.arg("vibrationWatchSeconds").toInt(), 1, 3600);
  cfg.sendTimeout = constrain(server.arg("sendTimeout").toInt(), 10, 1800);
  cfg.minSat = constrain(server.arg("minSat").toInt(), 0, 20);

  // Les paramètres MEMOIRE sont masqués de CONFIG : ne jamais les remettre à zéro
  // lorsque les champs ne sont pas présents dans le formulaire.
  if (server.hasArg("bufferMax"))
    cfg.bufferMax = constrain(server.arg("bufferMax").toInt(), 10, 7000);
  if (server.hasArg("recoveryRate"))
    cfg.recoveryRate = constrain(server.arg("recoveryRate").toInt(), 1, 20);

  cfg.energyMode = constrain(server.arg("energyMode").toInt(), 0, 5);

  cfg.vibrationEnable = server.hasArg("vibrationEnable");
  cfg.smsEnable = server.hasArg("smsEnable");
  cfg.ledEnable = server.hasArg("ledEnable");
  cfg.bufferEnable = true; // Le stockage des éléments est permanent et ne peut pas être désactivé.
  if (server.hasArg("transmitMode")) cfg.transmitMode = constrain(server.arg("transmitMode").toInt(), 0, 1);
  // Calibration batterie reste interne et n'est pas affichée dans CONFIG.
  if (server.hasArg("batteryCalibration")) { float bc = server.arg("batteryCalibration").toFloat(); if (bc >= 0.5f && bc <= 1.5f) cfg.batteryCalibration = bc; }
  // Les paramètres techniques non affichés dans CONFIG restent conservés.
  if (server.hasArg("batteryAction")) cfg.batteryAction = constrain(server.arg("batteryAction").toInt(), 0, 2);
  if (server.hasArg("vibrationSensitivity")) {
    String vs=server.arg("vibrationSensitivity");
    if(vs=="LOW") cfg.vibrationThreshold=2;
    else if(vs=="HIGH") cfg.vibrationThreshold=10;
    else cfg.vibrationThreshold=5;
  }

  if (server.hasArg("smsAdmin1")) cfg.smsAdmin1 = server.arg("smsAdmin1");
  if (server.hasArg("smsAdmin2")) cfg.smsAdmin2 = server.arg("smsAdmin2");
  if (server.hasArg("smsAdmin3")) cfg.smsAdmin3 = server.arg("smsAdmin3");
  if (server.hasArg("smsPassword")) { String p = server.arg("smsPassword"); if (p.length() >= 4 && p.length() <= 16 && p.indexOf(' ') < 0 && p.indexOf('_') < 0) cfg.smsPassword = p; }
  if (server.hasArg("smsPrivateCode")) { String p = server.arg("smsPrivateCode"); if (p.length() >= 4 && p.length() <= 16 && p.indexOf(' ') < 0 && p.indexOf('_') < 0) cfg.smsPrivateCode = p; }

  cfg.mqttEnable = server.hasArg("mqttEnable");
  if (server.hasArg("mqttServer")) cfg.mqttServer = server.arg("mqttServer");
  if (server.hasArg("mqttPort")) cfg.mqttPort = constrain(server.arg("mqttPort").toInt(), 1, 65535);
  if (server.hasArg("mqttUser")) cfg.mqttUser = server.arg("mqttUser");
  if (server.hasArg("mqttPassword")) cfg.mqttPassword = server.arg("mqttPassword");
  if (server.hasArg("mqttKeepAlive")) cfg.mqttKeepAlive = constrain(server.arg("mqttKeepAlive").toInt(), 10, 3600);
  if (server.hasArg("mqttReplyNumber")) {
    String n = normalizePhone(server.arg("mqttReplyNumber"));
    if (n.length() == 0 || validPhoneNumber(n)) cfg.mqttReplyNumber = n;
  }

  saveConfig();
  mqttStopService();
  mqttLastAttempt = 0;

  // Appliquer immediatement les nouveaux parametres des deux serveurs.
  // La logique TCP d'envoi reste inchangée : seul le parametre du lien
  // est rafraichi ici apres une modification depuis le Web.
  initLinks();

  if (oldServer2 != cfg.server2 || oldPort2 != cfg.port2 ||
      oldServer2Enable != cfg.server2Enable) {
    link2.connected = false;
    link2.opening = false;
    link2.openingSince = 0;
    link2.sending = false;
    link2.loginSent = false;
    link2.lastConnectAttempt = 0;
  }

  server.sendHeader("Location", "/config");
  server.send(303);
}

// =========================
// PAGES WEB OPTIMISÉES
// =========================
void simplePage(const String &title,const String &content){webBegin(title);server.sendContent(F("<div class='section'>"));server.sendContent(content);server.sendContent(F("</div>"));webEnd();}

void pageGPS(){
  String c;c.reserve(3600);
  c+=F("<div class='grid'><div class='card'><h3>LOCALISATION</h3><div id='gp' class='value'>--</div></div><div class='card'><h3>POSITION</h3><div id='fix' class='value'>--</div><div id='fq' class='small'>--</div></div><div class='card'><h3>SATELLITES</h3><div id='sat' class='value'>--</div></div><div class='card'><h3>LATITUDE</h3><div id='lat' class='value'>--</div></div><div class='card'><h3>LONGITUDE</h3><div id='lon' class='value'>--</div></div><div class='card'><h3>ALTITUDE</h3><div id='alt' class='value'>--</div></div><div class='card'><h3>VITESSE</h3><div id='spd' class='value'>--</div></div><div class='card'><h3>DIRECTION</h3><div id='course' class='value'>--</div></div><div class='card'><h3>DATE / HEURE</h3><div id='time' class='value'>--</div></div></div><div class='section' style='text-align:center'><h3>CARTE</h3><a id='mapLink' href='#' target='_blank'><button id='mapBtn' disabled>OUVRIR LA POSITION</button></a><button onclick='location.href=&quot;/gps/on&quot;'>ACTIVER LOCALISATION</button><button onclick='location.href=&quot;/gps/off&quot;'>DÉSACTIVER LOCALISATION</button></div>");
  c+=F(R"rawliteral(<script>
const $=id=>document.getElementById(id);function refreshGPS(){fetch('/api/gps?x='+Date.now(),{cache:'no-store'}).then(r=>r.json()).then(d=>{$('gp').textContent=d.power?'ACTIVE':'VEILLE';$('fix').textContent=d.fix?'FIX':'NO FIX';$('fq').textContent='Qualité du signal : '+d.fixQuality;$('sat').textContent=d.satellites;$('lat').textContent=Number(d.lat).toFixed(6);$('lon').textContent=Number(d.lon).toFixed(6);$('alt').textContent=Number(d.altitude).toFixed(1)+' m';$('spd').textContent=Number(d.speed).toFixed(1)+' km/h';$('course').textContent=Number(d.course).toFixed(0)+'°';$('time').textContent=d.utc+' / '+d.date;let b=$('mapBtn'),a=$('mapLink');if(d.fix){a.href='https://maps.google.com/?q='+Number(d.lat).toFixed(6)+','+Number(d.lon).toFixed(6);b.disabled=false}else{a.href='#';b.disabled=true}}).catch(()=>{})}refreshGPS();setInterval(refreshGPS,3000);
</script>)rawliteral");
  simplePage("GPS",c);
}

void pageNetwork(){
  String c;c.reserve(3000);
  c+=F("<div class='section'><h3>CHAÎNE DE COMMUNICATION</h3><div class='grid'><div class='card'><h3>COMMUNICATION</h3><div class='value ");c+=(modemPower && simReady && networkRegistered && tcpServiceOpen)?F("good'>ACTIVE"):F("red'>INACTIVE");c+=F("</div><div class='small'>");c+=modemState;c+=F("</div></div><div class='card'><h3>CARTE SIM</h3><div class='value ");c+=simPresent?F("good'>SIM"):F("red'>ABSENTE");c+=F("</div><div class='small'>");c+=simState;c+=F("</div></div><div class='card'><h3>RÉSEAU</h3><div class='value ");c+=networkRegistered?F("good'>ENREGISTRÉ"):F("red'>NON ENREGISTRÉ");c+=F("</div><div class='small'>");c+=networkType;c+=F("</div></div><div class='card'><h3>ACCÈS INTERNET</h3><div class='value ");c+=tcpServiceOpen?F("good'>DISPONIBLE"):F("red'>INDISPONIBLE");c+=F("</div></div><div class='card'><h3>SIGNAL</h3><div class='value'>");if(networkRSSI>=0&&networkRSSI<=31)c+=String(networkRSSI)+F(" / 31");else c+=F("—");c+=F("</div></div><div class='card'><h3>OPÉRATEUR</h3><div class='value' style='font-size:17px'>");c+=networkOperator.length()?networkOperator:F("—");c+=F("</div></div></div></div>");
  c+=F("<div class='section'><h3>SERVICES DE TRANSMISSION</h3><div class='grid'><div class='card'><h3>SERVEUR 1</h3><div class='value ");c+=link1.connected?F("good'>CONNECTÉ"):F("red'>DÉCONNECTÉ");c+=F("</div><div class='small mono'>");c+=cfg.server1;c+=F(":");c+=String(cfg.port1);c+=F("</div></div><div class='card'><h3>SERVEUR 2</h3><div class='value ");c+=link2.connected?F("good'>CONNECTÉ"):F("red'>DÉCONNECTÉ");c+=F("</div><div class='small mono'>");c+=cfg.server2;c+=F(":");c+=String(cfg.port2);c+=F("</div></div></div><div class='small'>État de transmission : ");c+=tcpServiceOpen?F("ACTIVE"):F("EN ATTENTE");c+=F(" · IMEI : ");c+=cfg.imei;c+=F("</div></div><div class='actions'><button onclick='location.href=&quot;/modem/on&quot;'>ACTIVER LA COMMUNICATION</button><button onclick='location.href=&quot;/modem/off&quot;'>DÉSACTIVER LA COMMUNICATION</button></div>");
  simplePage("Réseau",c);
}

void pageBuffer(){
  const uint32_t cap=cfg.bufferMax?cfg.bufferMax:1;const uint32_t pct=min<uint32_t>(100,(bufferPending*100UL)/cap);
  String c;c.reserve(2200);c+=F("<div class='grid'><div class='card'><h3>EN ATTENTE</h3><div class='value'>");c+=String(bufferPending);c+=F("</div></div><div class='card'><h3>ENVOYÉES</h3><div class='value'>");c+=String(bufferSent);c+=F("</div></div><div class='card'><h3>TENTATIVES</h3><div class='value'>");c+=String(bufferRetry);c+=F("</div></div><div class='card'><h3>PERDUES</h3><div class='value ");c+=bufferLost?F("red'>"):F("good'>");c+=String(bufferLost);c+=F("</div></div></div><div class='section'><h3>OCCUPATION</h3><div class='progress'><i style='width:");c+=String(pct);c+=F("%'></i></div><div class='progressText'><span>");c+=String(bufferPending);c+=F(" / ");c+=String(cfg.bufferMax);c+=F(" trames</span><span>");c+=String(pct);c+=F(" %</span></div></div><div class='section'><h3>PARAMÈTRES</h3><div class='grid'><div><b>Position</b><br>");c+=String(cfg.positionInterval);c+=F(" s</div><div><b>Grand intervalle</b><br>");c+=String(cfg.longIntervalMin);c+=F(" min</div><div><b>Récupération</b><br>");c+=String(cfg.recoveryRate);c+=F(" /s</div><div><b>Capacité</b><br>");c+=String(cfg.bufferMax);c+=F(" trames</div></div></div>");
  simplePage("Mémoire",c);
}

void apiGPS(){
  server.sendHeader("Cache-Control","no-store");StaticJsonDocument<384> d;
  d["power"]=gpsPower;d["fix"]=gps.fix;d["fixQuality"]=gps.fixQuality;d["rmcStatus"]=String(gps.rmcStatus);d["satellites"]=gps.satellites;d["lat"]=gps.lat;d["lon"]=gps.lon;d["altitude"]=gps.altitude;d["speed"]=gps.speed;d["course"]=gps.course;d["utc"]=gps.utc;d["date"]=gps.date;String out;serializeJson(d,out);server.send(200,"application/json",out);
}

void apiTrace(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"text/plain; charset=utf-8","JOURNAUX DESACTIVES - VERSION LEGERE");
}

void clearTrace(){
  server.send(200,"text/plain; charset=utf-8","OK - JOURNAUX DESACTIVES");
}

void apiOTAStatus(){
  server.sendHeader("Cache-Control","no-store");
  String json="{";
  json += "\"active\":" + String(otaOTAInProgress ? "true":"false");
  json += ",\"percent\":" + String(otaProgressPercent);
  json += ",\"received\":" + String(otaBytesReceived);
  json += ",\"total\":" + String(otaTotalBytes);
  json += ",\"stage\":\"";
  String st=otaStage; st.replace("\\","\\\\"); st.replace("\"","\\\""); st.replace("\n"," ");
  json += st + "\"";
  json += ",\"error\":\"";
  String er=otaLastError; er.replace("\\","\\\\"); er.replace("\"","\\\"");
  json += er + "\"}";
  server.send(200,"application/json; charset=utf-8",json);
}

void pageDiagnostic(){
  String c; c.reserve(7600);
  c+=F(R"rawliteral(<div class='section'><div class='sectionHead'><div><h3>MONITEUR DES TRANSMISSIONS</h3><div class='small'>Visualisation en temps réel des échanges. Le journal reste uniquement en mémoire.</div></div><span id='state' class='pill ok'>SURVEILLANCE</span></div><div class='actions'><button onclick="selectTrace('modem',this)">📡 COMMUNICATION 4G</button><button onclick="selectTrace('gps',this)">🛰 GPS</button><button onclick="selectTrace('ota',this)">MISE À JOUR</button><button onclick='clearCurrent()'>EFFACER</button></div><div class='small' id='sourceLabel'>Source : Communication 4G</div><pre id='traceBox' style='margin-top:12px;height:420px;overflow:auto;background:#050d17;border:1px solid #20344d;border-radius:10px;padding:12px;color:#b9d8ef;font-family:monospace;font-size:12px;white-space:pre-wrap;word-break:break-word'>Chargement...</pre></div><div class='grid'><div class='card'><h3>ÉTAT 4G</h3><div class='value' style='font-size:18px'>)rawliteral");
  c+=modemState; c+=F(R"rawliteral(</div><div class='small'>Réseau : )rawliteral"); c+=networkRegistered?F("ENREGISTRÉ"):F("NON ENREGISTRÉ"); c+=F(R"rawliteral( · Données : )rawliteral"); c+=tcpServiceOpen?F("ACTIVES"):F("INACTIVES");
  c+=F(R"rawliteral(</div></div><div class='card'><h3>GPS</h3><div class='value' style='font-size:18px'>)rawliteral"); c+=gps.fix?F("FIX"):F("NO FIX"); c+=F(R"rawliteral(</div><div class='small'>NMEA reçues : )rawliteral"); c+=String(nmeaRxCount); c+=F(R"rawliteral( · Valides : )rawliteral"); c+=String(nmeaValidCount);
  c+=F(R"rawliteral(</div></div><div class='card'><h3>SERVEUR</h3><div class='value' style='font-size:18px'>S1 )rawliteral"); c+=link1.connected?F("OK"):F("—"); c+=F(" / S2 "); c+=link2.connected?F("OK"):F("—");
  c+=F(R"rawliteral(</div><div class='small'>Communication de suivi</div></div><div class='card'><h3>SERVEUR CMD / MQTT</h3><div class='value' style='font-size:18px'>)rawliteral"); c+=(!cfg.mqttEnable)?F("DÉSACTIVÉ"):((mqttConnected&&mqttSubscribed)?F("CONNECTÉ"):F("EN ATTENTE")); c+=F(R"rawliteral(</div><div class='small'>Dernière commande : )rawliteral"); c+=mqttLastCommand.length()?mqttLastCommand:F("—");
  c+=F(R"rawliteral(</div></div></div><div class='section'><div class='sectionHead'><h3>PROGRESSION MISE À JOUR</h3><span id='otapill' class='pill'>EN ATTENTE</span></div><div style='height:18px;background:#0b1724;border:1px solid #20344d;border-radius:9px;overflow:hidden'><i id='otabar' style='display:block;height:100%;width:0%;background:#39a0ff'></i></div><div class='small' id='otastage' style='margin-top:8px'>Aucune mise à jour en cours.</div><div class='small' id='otabytes'>0 / 0 octets</div></div><form action='/save' method='POST'><div class='section'><div class='sectionHead'><h3>CONFIGURATION SERVEUR CMD / MQTT</h3><span class='pill )rawliteral"); c+=cfg.mqttEnable?F("ok'>ACTIF"):F("bad'>INACTIF");
  c+=F(R"rawliteral(</span></div><label><input type='checkbox' name='mqttEnable' )rawliteral"); if(cfg.mqttEnable)c+=F("checked"); c+=F(R"rawliteral(>Activer les commandes distantes</label><div class='row'><div><label>Adresse du service</label><input name='mqttServer' autocomplete='off' value=')rawliteral"); c+=cfg.mqttServer; c+=F(R"rawliteral('></div><div><label>Port</label><input type='number' name='mqttPort' min='1' max='65535' value=')rawliteral"); c+=String(cfg.mqttPort); c+=F(R"rawliteral('></div><div><label>Identifiant</label><input name='mqttUser' autocomplete='off' value=')rawliteral"); c+=cfg.mqttUser; c+=F(R"rawliteral('></div><div><label>Mot de passe du service</label><input name='mqttPassword' type='password' value=')rawliteral"); c+=cfg.mqttPassword; c+=F(R"rawliteral('></div><div><label>Maintien de liaison (s)</label><input type='number' name='mqttKeepAlive' min='10' max='3600' value=')rawliteral"); c+=String(cfg.mqttKeepAlive); c+=F(R"rawliteral('></div><div><label>Numéro SMS de réponse MQTT</label><input name='mqttReplyNumber' value=')rawliteral"); c+=cfg.mqttReplyNumber; c+=F(R"rawliteral(' placeholder='Vide = premier administrateur'></div></div><div class='small'>Canal de commande : )rawliteral"); c+=mqttCmdTopic();
  c+=F(R"rawliteral(<br>La configuration MQTT est disponible uniquement dans DIAGNOSTIC.</div></div><div class='section'><div class='sectionHead'><h3>ACCÈS PRIVÉ SMS</h3><span class='pill ok'>PROTÉGÉ</span></div><div class='row'><div><label>Mot de passe SMS principal</label><input name='smsPassword' type='password' value=')rawliteral"); c+=cfg.smsPassword; c+=F(R"rawliteral('></div><div><label>Code privé SMS</label><input name='smsPrivateCode' type='password' minlength='4' maxlength='16' value=')rawliteral"); c+=cfg.smsPrivateCode; c+=F(R"rawliteral('></div></div><div class='small'>Le code privé permet l'authentification d'une commande SMS sans utiliser un numéro administrateur. Conservez-le confidentiel.</div></div><div class='section'><button type='submit'>ENREGISTRER MQTT / ACCÈS PRIVÉ</button></div></form><script>let current='modem',timer=null,last='';const box=document.getElementById('traceBox'),label=document.getElementById('sourceLabel');function selectTrace(s,b){current=s;label.textContent='Source : '+(s==='modem'?'Communication 4G':s==='gps'?'GPS':'Mise à jour 4G');loadTrace();}async function loadTrace(){try{const r=await fetch('/api/trace?src='+current,{cache:'no-store'});const t=await r.text();if(t!==last){const at=box.scrollTop+box.clientHeight>=box.scrollHeight-30;box.textContent=t||'Aucune transmission enregistrée.';last=t;if(at)box.scrollTop=box.scrollHeight}}catch(e){document.getElementById('state').textContent='HORS LIGNE'}}async function clearCurrent(){await fetch('/api/trace/clear?src='+current);last='';loadTrace();}loadTrace();timer=setInterval(loadTrace,700);async function loadOTA(){try{const r=await fetch('/api/ota-status',{cache:'no-store'});const x=await r.json();document.getElementById('otabar').style.width=x.percent+'%';document.getElementById('otastage').textContent=x.stage||'En attente';document.getElementById('otabytes').textContent=(x.received||0)+' / '+(x.total||0)+' octets';const p=document.getElementById('otapill');p.textContent=x.active?'EN COURS':(x.error?'ERREUR':'EN ATTENTE');p.className='pill '+(x.active?'ok':(x.error?'bad':''));}catch(e){}}loadOTA();setInterval(loadOTA,700);</script>)rawliteral");
  simplePage("Diagnostic",c);
}

void pageUpdateFile(){
  String c;c.reserve(2400);c+=F(R"rawliteral(<div class='section'><h3>MISE À JOUR PAR FICHIER</h3><div class='small'>Sélectionnez le fichier de mise à jour depuis votre ordinateur.</div><input id='fwfile' type='file' accept='.bin,application/octet-stream'><button id='filebtn' onclick='startFileUpdate()'>INSTALLER LA MISE À JOUR</button><div class='progress'><i id='upbar' style='width:0%'></i></div><div id='upmsg' class='small'>En attente...</div><pre id='atlog' class='section'></pre></div><script>
const m=document.getElementById('upmsg'),b=document.getElementById('upbar'),l=document.getElementById('atlog');async function startFileUpdate(){const f=document.getElementById('fwfile').files[0];if(!f){m.textContent='Sélectionnez un fichier .bin';return}if(!/\.bin$/i.test(f.name)){m.textContent='Fichier invalide';return}if(!confirm('Programmer ce firmware ?'))return;document.getElementById('filebtn').disabled=true;try{const x=new XMLHttpRequest();x.open('POST','/update-file');x.upload.onprogress=e=>{if(e.lengthComputable){let p=e.loaded*100/e.total;b.style.width=p+'%';m.textContent='Envoi : '+Math.round(p)+'%'}};x.onload=()=>{l.textContent=x.responseText;const ok=/SUCCES|Fichier reçu/i.test(x.responseText);m.textContent=ok?'Mise à jour réussie':'Erreur';document.getElementById('filebtn').disabled=false};x.onerror=()=>{m.textContent='Erreur de connexion';document.getElementById('filebtn').disabled=false};let fd=new FormData();fd.append('firmware',f,f.name);x.send(fd)}catch(e){m.textContent=e.message;document.getElementById('filebtn').disabled=false}}</script>)rawliteral");simplePage("Mise à jour",c);
}

void pageUpdate4G(){
  // Accès privé séparé du diagnostic : cette page n'est pas affichée dans la navigation.
  // La console de transmission reste visible car elle est utile pour suivre la mise à jour.
  webBegin("Mise à jour 4G");
  server.sendContent(F(R"rawliteral(
<div class='section' style='max-width:900px;margin:auto'>
  <div class='sectionHead'>
    <div>
      <h3>MISE À JOUR 4G</h3>
      <div class='small'>Mise à jour à distance du logiciel.</div>
    </div>
    <span id='pill' class='pill ok'>PRÊT</span>
  </div>

  <div class='row'>
    <div style='flex:1'>
      <label>Version</label>
      <input id='fwver' type='text' placeholder='Exemple : V10.00' autocomplete='off'>
    </div>
  </div>

  <div class='actions'>
    <button id='btn' onclick='startUpdate()'>METTRE À JOUR</button>
  </div>

  <div id='wait' style='display:none;text-align:center;padding:18px 0'>
    <div class='otaWait'></div>
    <div id='msg' class='small' style='margin-top:10px'>Mise à jour en cours...</div>
  </div>

  <div id='readyMsg' class='small' style='margin-top:10px'>Prêt.</div>

  <div class='section'>
    <div class='sectionHead'>
      <h3>CONSOLE DE TRANSMISSION</h3>
      <button type='button' onclick='clearConsole()'>EFFACER</button>
    </div>
    <pre id='console' style='height:300px;overflow:auto;margin-top:10px;padding:12px;background:#050d18;border:1px solid #20344d;border-radius:9px;color:#9fb3c8;font-size:11px;line-height:1.45;white-space:pre-wrap'>Aucune transmission.</pre>
  </div>
</div>

<style>
.otaWait{
  width:34px;height:34px;margin:auto;
  border:4px solid #20344d;border-top-color:#38bdf8;
  border-radius:50%;animation:otaSpin .8s linear infinite
}
@keyframes otaSpin{to{transform:rotate(360deg)}}
</style>

<script>
const btn=document.getElementById('btn');
const msg=document.getElementById('msg');
const readyMsg=document.getElementById('readyMsg');
const wait=document.getElementById('wait');
const pill=document.getElementById('pill');
const box=document.getElementById('console');
let busy=false,lastTrace='';

function setState(t,active){
  pill.textContent=active?'EN COURS':t;
  pill.className='pill '+(active?'ok':(t==='ERREUR'?'bad':'ok'));
}

async function refresh(){
  try{
    const r=await fetch('/api/ota-status',{cache:'no-store'});
    const x=await r.json();

    if(x.active){
      busy=true;
      btn.disabled=true;
      wait.style.display='block';
      readyMsg.style.display='none';
      setState('EN COURS',true);
      msg.textContent=x.stage||'Mise à jour en cours...';
    }else if(x.error){
      busy=false;
      btn.disabled=false;
      wait.style.display='none';
      readyMsg.style.display='block';
      readyMsg.textContent='La mise à jour a échoué.';
      setState('ERREUR',false);
    }else if(x.stage && /termin|réuss|succes/i.test(x.stage)){
      wait.style.display='block';
      readyMsg.style.display='none';
      setState('TERMINÉE',false);
      msg.textContent=x.stage;
    }else if(!busy){
      wait.style.display='none';
      readyMsg.style.display='block';
      setState('PRÊT',false);
    }
  }catch(e){}
}

async function refreshConsole(){
  try{
    const r=await fetch('/api/trace?src=ota',{cache:'no-store'});
    const t=await r.text();
    if(t!==lastTrace){
      const nearBottom=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
      box.textContent=t||'Aucune transmission.';
      lastTrace=t;
      if(nearBottom)box.scrollTop=box.scrollHeight;
    }
  }catch(e){}
}

async function clearConsole(){
  try{
    await fetch('/api/trace/clear?src=ota',{cache:'no-store'});
    lastTrace='';
    refreshConsole();
  }catch(e){}
}

async function startUpdate(){
  if(busy)return;
  const v=document.getElementById('fwver').value.trim();
  if(!v || /[:\\/\\s]/.test(v)){
    readyMsg.textContent='Version invalide.';
    return;
  }
  if(!confirm('Installer la version '+v+' ?'))return;

  busy=true;
  btn.disabled=true;
  wait.style.display='block';
  readyMsg.style.display='none';
  setState('EN COURS',true);
  msg.textContent='Préparation...';

  try{
    const rr=await fetch('/update-url',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'version='+encodeURIComponent(v)
    });
    await rr.text();
  }catch(e){
    busy=false;
    btn.disabled=false;
    wait.style.display='none';
    readyMsg.style.display='block';
    readyMsg.textContent='La communication a été interrompue.';
    setState('ERREUR',false);
  }
}

refresh();
refreshConsole();
setInterval(refresh,1000);
setInterval(refreshConsole,500);
</script>
)rawliteral"));
  webEnd();
}

// =========================
// ROUTES WEB SANS REDIRECTION CAPTIVE
// =========================
void setupRoutes() {

  // Page d'accueil = configuration (chargement direct après connexion)
  server.on("/style.css", HTTP_GET, []() { server.send_P(200, "text/css; charset=utf-8", WEB_CSS); });
  server.on("/", pageInfo);
  server.on("/info", pageInfo);
  server.on("/config", pageConfig);
  server.on("/gps", pageGPS);
  server.on("/api/gps", HTTP_GET, apiGPS);
  server.on("/network", pageNetwork);
  server.on("/buffer", pageBuffer);
  server.on("/update", HTTP_GET, pageUpdateFile);
  server.on("/update-file-page", HTTP_GET, pageUpdateFile);
  server.on("/mise-a-jour-4g", HTTP_GET, pageUpdate4G);
  server.on("/diagnostique", HTTP_GET, pageDiagnostic);
  server.on("/diagnostic", HTTP_GET, pageDiagnostic);
  server.on("/api/trace", HTTP_GET, apiTrace);
  server.on("/api/trace/clear", HTTP_GET, clearTrace);
  server.on("/api/ota-status", HTTP_GET, apiOTAStatus);

  server.on("/update-url", HTTP_POST, []() {
    String version = server.arg("version");
    version.trim();
    String url = buildGitHubFirmwareURL(version);
    if (!url.length()) {
      server.send(400, "text/plain; charset=utf-8", "ERREUR : VERSION INVALIDE\n");
      return;
    }

    // Lancer l'OTA hors du handler HTTP. La procedure peut durer plusieurs
    // minutes ; elle doit laisser /api/trace et /api/ota-status fonctionner
    // pendant toute la reception du firmware.
    if (otaOTAInProgress || otaWebRequestPending) {
      server.send(409, "text/plain; charset=utf-8", "MISE A JOUR DEJA EN COURS\n");
      return;
    }

    cleanupTemporaryFiles();
    otaWebRequestURL = url;
    otaWebRequestPending = true;
    otaLastError = "";
    otaStatusMessage = "Préparation de la mise à jour...";
    otaStage = "Préparation";
    otaBytesReceived = 0;
    otaTotalBytes = 0;
    otaProgressPercent = 0;
    otaWebLog("VERSION : " + version);
    otaWebLog("URL : " + url);

    // Réponse immédiate : le navigateur peut maintenant interroger la
    // console et l'état pendant que l'OTA s'exécute dans la boucle principale.
    server.send(202, "text/plain; charset=utf-8",
                "MISE A JOUR PROGRAMMEE\nVERSION : " + version + "\n");
  });

  server.on("/update-file", HTTP_POST, []() {
    if (otaUploadFailed || !otaLocalDirectMode || !otaLocalUpdateStarted) {
      if (otaLocalUpdateStarted) Update.end(false);
      otaLocalUpdateStarted = false;
      otaLocalDirectMode = false;
      server.send(500, "text/plain; charset=utf-8",
                  "ERREUR : transfert interrompu ou réception OTA impossible\nCODE : " + otaLastError + "\n");
      return;
    }

    otaWebStreamActive = true;
    otaWebLog("Réception terminée : validation du firmware...");
    bool ok = otaFinishCommon(otaLocalUploadValidator);
    otaLocalUpdateStarted = false;
    otaLocalDirectMode = false;
    otaWebStreamActive = false;

    if (ok) {
      otaProgressPercent = 100;
      otaStatusMessage = "Firmware installé - redémarrage";
      otaRestartPending = true;
      otaRestartAt = millis() + 1500UL;
      server.send(200, "text/plain; charset=utf-8",
                  "Fichier reçu : " + String(otaUploadReceived) +
                  " octets\nSUCCES : firmware valide et installé\nREDÉMARRAGE...\n");
    } else {
      server.send(500, "text/plain; charset=utf-8",
                  "ERREUR OTA\nCODE : " + otaLastError + "\n");
    }
  }, handleLocalFirmwareUpload);

  server.on("/save", HTTP_POST, saveFromWeb);

  server.on("/gps/on", []() {
    setGPSPower(true);
    server.sendHeader("Location", "/gps");
    server.send(303);
  });

  server.on("/gps/off", []() {
    setGPSPower(false);
    server.sendHeader("Location", "/gps");
    server.send(303);
  });

  server.on("/modem/on", []() {
    setModemPower(true);
    server.sendHeader("Location", "/network");
    server.send(303);
  });

  server.on("/modem/off", []() {
    setModemPower(false);
    server.sendHeader("Location", "/network");
    server.send(303);
  });

  server.on("/restart", []() {
    server.send(200, "text/html", "<h2>Redémarrage...</h2>");
    delay(300); ESP.restart();
  });

  server.on("/factory-reset", []() {
    server.send(200, "text/html", "<h2>Réinitialisation usine...</h2><p>Le tracker va redémarrer.</p>");
    delay(300); factoryReset();
  });

}


// =========================
// FILE SYSTEM / BUFFER SEGMENTE - FIFO
// Chaque fichier contient au maximum 100 trames.
// FIFO : la plus ancienne trame est envoyee en premier.
// Une trame n'est retiree qu'apres confirmation CIPSEND de tous
// les serveurs TCP requis (mecanisme existant conserve).
// =========================
void bufferFileName(uint16_t index, char *out, size_t outSize) {
  snprintf(out, outSize, "/trames_%04u.dat", (unsigned)index);
}

// Reconnaît uniquement les fichiers du buffer (pas les autres fichiers SPIFFS).
// kind: 1 = .dat, 2 = .dat.bak, 3 = .dat.tmp.
uint16_t countLinesInBufferFile(const char *path);
bool parseBufferFileName(const String &name, uint16_t &index, uint8_t &kind) {
  if (!name.startsWith("/trames_") || name.length() < 16) return false;
  uint32_t value = 0;
  for (uint8_t i = 8; i < 12; ++i) {
    char c = name[i];
    if (c < '0' || c > '9') return false;
    value = value * 10UL + (uint32_t)(c - '0');
  }
  if (value == 0 || value > BUFFER_MAX_FILE_INDEX) return false;
  const char *suffix = name.c_str() + 12;
  if (strcmp(suffix, ".dat") == 0) kind = 1;
  else if (strcmp(suffix, ".dat.bak") == 0) kind = 2;
  else if (strcmp(suffix, ".dat.tmp") == 0) kind = 3;
  else return false;
  index = (uint16_t)value;
  return true;
}

uint16_t findOldestBufferFileIndex();

uint16_t countLinesInBufferFile(const char *path) {
  File f = SPIFFS.open(path, "r");
  if (!f) return 0;
  uint16_t count = 0, byteCounter = 0;
  while (f.available()) {
    if (f.read() == '\n' && count < 65535) count++;
    if (++byteCounter >= 256) { byteCounter = 0; yield(); }
  }
  f.close();
  return count;
}

void bufferCursorName(uint16_t index, char *out, size_t outSize) {
  snprintf(out, outSize, "/trames_%04u.pos", (unsigned)index);
}

uint16_t readBufferCursor(uint16_t index) {
  char path[28];
  bufferCursorName(index, path, sizeof(path));
  File f = SPIFFS.open(path, "r");
  if (!f) return 0;
  String value = f.readStringUntil('\n');
  f.close();
  long n = value.toInt();
  if (n < 0) return 0;
  if (n > (long)BUFFER_FILE_CAPACITY) n = BUFFER_FILE_CAPACITY;
  return (uint16_t)n;
}

// Conversion unique du curseur historique (nombre de lignes) en offset octet.
// Ce scan n'est fait qu'au chargement/changement de fichier, jamais a chaque trame.
uint32_t bufferLineCursorToByteOffset(const char *path, uint16_t linesToSkip) {
  if (!linesToSkip) return 0;
  File f = SPIFFS.open(path, "r");
  if (!f) return 0;
  uint32_t offset = 0;
  uint16_t skipped = 0;
  while (f.available() && skipped < linesToSkip) {
    int c = f.read();
    if (c < 0) break;
    offset++;
    if (c == '\n') skipped++;
    if ((offset & 0xFFU) == 0) yield();
  }
  f.close();
  return offset;
}

// Le curseur est un petit fichier : on ne reecrit plus les 99 autres trames.
// En cas de coupure pendant sa mise a jour, un doublon est possible, jamais
// une suppression prematuree d'une trame non confirmee.
bool writeBufferCursor(uint16_t index, uint16_t offset) {
  char path[28], tmp[36];
  bufferCursorName(index, path, sizeof(path));
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  if (SPIFFS.exists(tmp)) SPIFFS.remove(tmp);
  File f = SPIFFS.open(tmp, "w");
  if (!f) return false;
  size_t expected = String(offset).length() + 2U;
  size_t written = f.println(offset);
  f.flush();
  bool ok = (written == expected && f.size() == expected);
  f.close();
  if (!ok) { SPIFFS.remove(tmp); return false; }
  if (SPIFFS.exists(path)) SPIFFS.remove(path);
  if (!SPIFFS.rename(tmp, path)) {
    if (SPIFFS.exists(tmp)) SPIFFS.remove(tmp);
    return false;
  }
  return true;
}

// Selectionne le prochain fichier FIFO une seule fois au changement de fichier.
bool selectNextOldestBufferFile(uint16_t afterIndex = 0) {
  uint16_t candidate = 0;
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    uint16_t index = 0;
    uint8_t kind = 0;
    String name = dir.fileName();
    if (!parseBufferFileName(name, index, kind) || kind != 1 || index <= afterIndex) continue;
    if (candidate == 0 || index < candidate) candidate = index;
    yield();
  }
  if (!candidate) {
    bufferOldestFileIndex = 0;
    bufferOldestFileCount = 0;
    bufferOldestFileOffset = 0;
    bufferOldestByteOffset = 0;
    return false;
  }
  char path[24];
  bufferFileName(candidate, path, sizeof(path));
  uint16_t lines = countLinesInBufferFile(path);
  if (!lines) {
    SPIFFS.remove(path);
    return selectNextOldestBufferFile(candidate);
  }
  bufferOldestFileIndex = candidate;
  bufferOldestFileCount = lines;
  bufferOldestFileOffset = readBufferCursor(candidate);
  if (bufferOldestFileOffset > lines) bufferOldestFileOffset = 0;
  bufferOldestByteOffset = bufferLineCursorToByteOffset(path, bufferOldestFileOffset);
  return true;
}

bool finalizeOldestBufferFile() {
  if (!bufferOldestFileIndex) return false;
  if (bufferOldestFileOffset < bufferOldestFileCount) return true;
  uint16_t done = bufferOldestFileIndex;
  char path[24], pos[28];
  bufferFileName(done, path, sizeof(path));
  bufferCursorName(done, pos, sizeof(pos));
  if (SPIFFS.exists(path) && !SPIFFS.remove(path)) return false;
  if (SPIFFS.exists(pos)) SPIFFS.remove(pos);
  bufferOldestFileIndex = 0;
  bufferOldestFileCount = 0;
  bufferOldestFileOffset = 0;
  bufferOldestByteOffset = 0;
  return selectNextOldestBufferFile(done);
}

// Repare une operation de retrait interrompue par une coupure.
void recoverBufferFileTransaction(uint16_t index) {
  char path[24], bak[32], tmp[32];
  bufferFileName(index, path, sizeof(path));
  snprintf(bak, sizeof(bak), "%s.bak", path);
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  if (!SPIFFS.exists(path) && SPIFFS.exists(bak)) SPIFFS.rename(bak, path);
  if (SPIFFS.exists(path)) {
    if (SPIFFS.exists(bak)) SPIFFS.remove(bak);
    if (SPIFFS.exists(tmp)) SPIFFS.remove(tmp);
  } else if (SPIFFS.exists(tmp)) SPIFFS.remove(tmp);
}

bool appendBufferFrame(const String &frame) {
  if (!frame.length()) return false;
  if (bufferFileIndex == 0 || bufferFileCount >= BUFFER_FILE_CAPACITY) {
    if (bufferFileIndex >= BUFFER_MAX_FILE_INDEX) return false;
    bufferFileIndex++;
    bufferFileCount = 0;
  }
  char path[24];
  bufferFileName(bufferFileIndex, path, sizeof(path));
  File f = SPIFFS.open(path, "a");
  if (!f) return false;
  const size_t oldSize = f.size();
  const size_t expectedBytes = frame.length() + 2U;
  size_t written = f.println(frame);
  f.flush();
  size_t newSize = f.size();
  bool writeOK = (written == expectedBytes && newSize >= oldSize &&
                  (newSize - oldSize) == expectedBytes);
  if (!writeOK) { f.truncate(oldSize); f.flush(); }
  f.close();
  if (!writeOK) return false;
  bufferFileCount++;
  bufferPending++;
  // Si le fichier en cours d'ecriture est aussi le fichier FIFO lu,
  // actualiser son nombre de lignes pour ne jamais le finaliser trop tot.
  if (bufferOldestFileIndex == bufferFileIndex && bufferOldestFileIndex != 0) {
    bufferOldestFileCount++;
  }
  if (bufferOldestFileIndex == 0) {
    bufferOldestFileIndex = bufferFileIndex;
    bufferOldestFileCount = bufferFileCount;
    bufferOldestFileOffset = 0;
    bufferOldestByteOffset = 0;
  }
  return true;
}

void countBuffer() {
  bufferPending = 0;
  bufferFileIndex = 0;
  bufferFileCount = 0;
  bufferOldestFileIndex = 0;
  bufferOldestFileCount = 0;
  bufferOldestFileOffset = 0;
  bufferOldestByteOffset = 0;

  // Réparer d'abord les transactions interrompues. Une seule opération de
  // retrait peut être en cours à la fois : on recherche les .bak présents.
  for (uint8_t pass = 0; pass < 4; ++pass) {
    uint16_t backupIndex = 0;
    {
      Dir dir = SPIFFS.openDir("/");
      while (dir.next()) {
        uint16_t index = 0;
        uint8_t kind = 0;
        if (parseBufferFileName(dir.fileName(), index, kind) &&
            (kind == 2 || kind == 3)) {
          backupIndex = index;
          break;
        }
        yield();
      }
    }
    if (!backupIndex) break;
    recoverBufferFileTransaction(backupIndex);
    char repairPath[24], repairBak[32], repairTmp[32];
    bufferFileName(backupIndex, repairPath, sizeof(repairPath));
    snprintf(repairBak, sizeof(repairBak), "%s.bak", repairPath);
    snprintf(repairTmp, sizeof(repairTmp), "%s.tmp", repairPath);
    if (SPIFFS.exists(repairBak) || SPIFFS.exists(repairTmp)) break;
    yield();
  }

  // Comptage uniquement des fichiers réellement présents dans SPIFFS.
  // Cette méthode reste rapide même si les index comportent des trous.
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
      uint16_t index = 0;
      uint8_t kind = 0;
      String name = dir.fileName();
      if (!parseBufferFileName(name, index, kind) || kind != 1) continue;
      uint16_t n = countLinesInBufferFile(name.c_str());
      if (n == 0) continue;
      bufferPending += n;
      if (bufferOldestFileIndex == 0 || index < bufferOldestFileIndex) {
        bufferOldestFileIndex = index;
        bufferOldestFileCount = n;
      }
      if (index > bufferFileIndex) {
        bufferFileIndex = index;
        bufferFileCount = n;
      }
      yield();
    }
  }

  if (bufferOldestFileIndex) {
    bufferOldestFileOffset = readBufferCursor(bufferOldestFileIndex);
    if (bufferOldestFileOffset > bufferOldestFileCount) bufferOldestFileOffset = 0;
    char oldestPath[24];
    bufferFileName(bufferOldestFileIndex, oldestPath, sizeof(oldestPath));
    bufferOldestByteOffset = bufferLineCursorToByteOffset(oldestPath, bufferOldestFileOffset);
    if (bufferOldestFileOffset <= bufferPending) bufferPending -= bufferOldestFileOffset;
    if (bufferOldestFileOffset >= bufferOldestFileCount) {
      finalizeOldestBufferFile();
    }
  }

  // Migration de l'ancien buffer monolithique.
  if (SPIFFS.exists("/buffer.ndjson")) {
    File legacy = SPIFFS.open("/buffer.ndjson", "r");
    if (legacy) {
      bool migrationOK = true;
      while (legacy.available()) {
        String line = legacy.readStringUntil('\n');
        line.trim();
        if (line.length() && !appendBufferFrame(line)) { migrationOK = false; break; }
        yield();
      }
      legacy.close();
      if (migrationOK) SPIFFS.remove("/buffer.ndjson");
    }
  }
}

bool storePosition() {
  if (!gps.fix || cfg.imei.length() != 15) return false;
  String frame = buildGPS103Frame();
  if (!frame.length()) return false;
  FSInfo fsInfo;
  if (!SPIFFS.info(fsInfo)) { bufferLost++; return false; }
  const size_t freeBytes = (fsInfo.totalBytes > fsInfo.usedBytes)
                         ? (fsInfo.totalBytes - fsInfo.usedBytes) : 0;
  if (freeBytes <= BUFFER_RESERVED_SPACE ||
      freeBytes - BUFFER_RESERVED_SPACE < frame.length() + 2U) {
    bufferLost++;
    return false;
  }
  if (!appendBufferFrame(frame)) { bufferLost++; return false; }
  return true;
}

// FIFO optimise : lecture a partir du curseur, sans rescanner tous les fichiers.
bool peekBufferFrame(String &frame) {
  frame = "";
  if (!bufferPending) return false;
  if (!bufferOldestFileIndex && !selectNextOldestBufferFile()) return false;
  if (bufferOldestFileOffset >= bufferOldestFileCount) {
    if (!finalizeOldestBufferFile() || !bufferOldestFileIndex) return false;
  }

  char path[24];
  bufferFileName(bufferOldestFileIndex, path, sizeof(path));
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  if (!f.seek(bufferOldestByteOffset, SeekSet)) {
    f.close();
    return false;
  }
  frame = f.readStringUntil('\n');
  bufferPeekEndOffset = f.position();
  frame.trim();
  f.close();
  return frame.length() > 0;
}

// Apres confirmation CIPSEND, avance seulement un petit curseur persistant.
// Le fichier de 100 trames n'est supprime qu'une fois entierement consomme.
bool removeOldestBufferFrame() {
  if (!bufferPending || !bufferOldestFileIndex) return false;
  uint16_t nextOffset = bufferOldestFileOffset + 1;
  if (bufferPeekEndOffset <= bufferOldestByteOffset) return false;
  if (!writeBufferCursor(bufferOldestFileIndex, nextOffset)) return false;
  // Reprendre exactement apres le separateur lu : compatible LF ou CRLF.
  bufferOldestByteOffset = bufferPeekEndOffset;
  bufferPeekEndOffset = 0;
  bufferOldestFileOffset = nextOffset;
  bufferPending--;
  bufferSent++;

  if (bufferOldestFileOffset >= bufferOldestFileCount) {
    finalizeOldestBufferFile();
  }
  if (!bufferPending) {
    bufferFileIndex = 0;
    bufferFileCount = 0;
    bufferOldestFileIndex = 0;
    bufferOldestFileCount = 0;
    bufferOldestFileOffset = 0;
    bufferOldestByteOffset = 0;
  }
  return true;
}

void taskBufferMaintenance() {
  // Pas de compactage global : les fichiers restent limites a 100 trames.
}

// =========================
// NETWORK MANAGER
// =========================
void parseNetworkLine(const String& line) {
  String l = line;
  l.trim();
  String u = l;
  u.toUpperCase();

  if (!isValidIMEI(cfg.imei)) {
    String found = extractIMEIFromLine(l);
    if (found.length() == 15) {
      cfg.imei = found;
      modemIMEIRequested = true;
      saveConfig();
      addLog("IMEI AUTO: " + cfg.imei);
    }
  }

  if (u == "OK" && modemLastCommand == "AT") {
    modemATReady = true;
    modemState = "AT READY";
  }

  // Etat SIM : gérer à la fois la réponse CPIN normale et l'erreur
  // renvoyée par le modem lorsque la carte est absente.
  if (u.startsWith("+CPIN:")) {
    String v=l.substring(l.indexOf(':')+1); v.trim();
    simState=v;
    simReady=v.equalsIgnoreCase("READY");
    simPresent=!v.equalsIgnoreCase("NOT INSERTED") &&
               !v.equalsIgnoreCase("SIM ABSENT") &&
               v.length()>0;

    if (!simReady) {
      modemDataReady=false;
      networkRegistered=false;
      packetServiceAttached=false;
      tcpServiceOpen=false;
    }
  }

  // A7670 peut signaler une SIM retirée par :
  // +CME ERROR: SIM not inserted
  // ou +CME ERROR: 10
  // Il faut explicitement invalider les états précédents pour que
  // l'interface et la supervision ne conservent pas un ancien état.
  if (u.indexOf("+CME ERROR: SIM NOT INSERTED") >= 0 ||
      u.indexOf("+CME ERROR: 10") >= 0) {
    simPresent = false;
    simReady = false;
    simState = "SIM ABSENTE";
    modemDataReady = false;
    networkRegistered = false;
    packetServiceAttached = false;
    tcpServiceOpen = false;
    tcpNetOpening = false;
    networkRSSI = -1;
    networkOperator = "--";
    networkType = "--";
    link1.connected = link1.opening = link1.sending = link1.loginSent = false;
    link2.connected = link2.opening = link2.sending = link2.loginSent = false;
    modemState = "SIM ABSENTE";
    modemStage = MODEM_STAGE_SIM;
  }

  if (u.startsWith("+CSQ:")) {
    int p=l.indexOf(':'); String v=l.substring(p+1); v.trim(); int c=v.indexOf(',');
    if(c>0) networkRSSI=v.substring(0,c).toInt();
  }

  // Enregistrement reseau : CEREG est prioritaire pour LTE/A7670E.
  // Format requete : +CEREG: <n>,<stat>[,...]
  // Format URC     : +CEREG: <stat>[,"tac","ci",<AcT>]
  // Les champs TAC/CI/AcT ne doivent jamais etre interpretes comme le statut.
  if (u.startsWith("+CEREG:") || u.startsWith("+CGREG:")) {
    const bool isCereg = u.startsWith("+CEREG:");
    String value = l.substring(l.indexOf(':') + 1);
    value.trim();

    int firstComma = value.indexOf(',');
    String firstField = (firstComma >= 0)
      ? value.substring(0, firstComma) : value;
    firstField.trim();

    int status = firstField.toInt();
    bool queryFormat = false;

    if (firstComma >= 0) {
      String secondField = value.substring(firstComma + 1);
      int secondComma = secondField.indexOf(',');
      if (secondComma >= 0) secondField = secondField.substring(0, secondComma);
      secondField.trim();

      // Dans une reponse de requete, le deuxieme champ est le statut.
      // Dans une URC avec TAC/CI, le deuxieme champ commence par un guillemet.
      bool secondFieldIsNumeric = secondField.length() > 0;
      for (uint16_t i = 0; i < secondField.length(); i++) {
        if (!isDigit(secondField[i])) {
          secondFieldIsNumeric = false;
          break;
        }
      }
      if (secondFieldIsNumeric) {
        status = secondField.toInt();
        queryFormat = true;
      }
    }

    const bool registered = (status == 1 || status == 5);

    // CEREG represente l enregistrement LTE. Une indication CGREG moins
    // favorable ne doit pas annuler un enregistrement LTE deja confirme.
    if (isCereg || !ceregStatusKnown) {
      if (isCereg) ceregStatusKnown = true;
      networkRegistered = registered;

      if (!registered) {
        modemDataReady = false;
        packetServiceAttached = false;
        tcpServiceOpen = false;
        tcpNetOpening = false;
        link1.connected = link1.opening = link1.sending = link1.loginSent = false;
        link2.connected = link2.opening = link2.sending = link2.loginSent = false;
        modemStage = MODEM_STAGE_REGISTRATION;
      } else if (simReady && modemStage < MODEM_STAGE_DATA) {
        modemStage = MODEM_STAGE_DATA;
      }

      addLog(String(isCereg ? "CEREG" : "CGREG") +
             String(queryFormat ? " QUERY STAT=" : " URC STAT=") +
             String(status) + (registered ? " ENREGISTRE" : " NON ENREGISTRE"));
    }
  }

  if (u.startsWith("+CGATT:")) {
    // Information uniquement : CGATT n'est plus un verrou TCP.
    packetServiceAttached =
      (l.substring(l.indexOf(':')+1).toInt()==1) && simReady && networkRegistered;
  }

  if (u.startsWith("+COPS:")) {
    int q1=l.lastIndexOf('"'); int q0=l.lastIndexOf('"',q1-1);
    if(q0>=0 && q1>q0) networkOperator=l.substring(q0+1,q1);
  }

  if (u.startsWith("+CPSI:")) {
    String v=l.substring(l.indexOf(':')+1); v.trim(); int comma=v.indexOf(',');
    networkType=(comma>0)?v.substring(0,comma):v;
  }

  // Un ERROR synchrone à NETOPEN doit libérer l'état d'attente.
  // La prochaine tentative sera autorisée après NET_REOPEN_INTERVAL_MS.
  if (u == "ERROR" && modemLastCommand == "AT+NETOPEN") {
    tcpNetOpening = false;
    tcpServiceOpen = false;
    modemDataReady = false;
    modemWaitingResponse = false;
    modemStage = MODEM_STAGE_DATA;
    netOpenFailureCount++;
    addLog("NETOPEN ERROR - retry delayed");
  }

  if (u.startsWith("+NETOPEN:")) {
    String v=l.substring(l.indexOf(':')+1); v.trim();
    int err=v.toInt();
    tcpNetOpening=false;
    // NETOPEN confirme le service IP. CGATT n'est pas utilisé comme
    // condition logicielle supplémentaire : SIM + réseau suffisent pour tenter.
    tcpServiceOpen=(err==0) && simReady && networkRegistered;
    modemWaitingResponse=false;
    if (tcpServiceOpen) {
      modemDataReady=true;
      modemStage=MODEM_STAGE_READY;
      modemRecoveryCount=0;
      netOpenFailureCount=0;
    } else {
      modemDataReady=false;
      modemStage=MODEM_STAGE_DATA;
      // Conserver lastNetOpenAttempt pour respecter le délai avant retry.

      // Echec NETOPEN : conserver le modem allumé et réessayer
      // périodiquement. Aucun power-cycle automatique après plusieurs échecs.
      netOpenFailureCount++;
    }
  }

  if (u.startsWith("+NETCLOSE:")) {
    tcpNetOpening=false; tcpServiceOpen=false; modemDataReady=false;
    modemStage=MODEM_STAGE_DATA;
    lastNetOpenAttempt=0;
    link1.connected=false; link1.opening=false; link1.openingSince=0; link1.sending=false; link1.loginSent=false;
    link2.connected=false; link2.opening=false; link2.openingSince=0; link2.sending=false; link2.loginSent=false;
    addLog("NETCLOSE -> AUTO RECOVERY");
  }

  if (u.startsWith("+CIPOPEN:")) {
    int p=l.indexOf(':'); String v=l.substring(p+1); v.trim();
    int comma=v.indexOf(',');
    if(comma>0) {
      int sock=v.substring(0,comma).toInt();
      int err=v.substring(comma+1).toInt();
      TcpLinkState *ln = sock==1 ? &link1 : (sock==2 ? &link2 : nullptr);
      if(ln) {
        ln->opening=false;
        ln->openingSince=0;
        ln->connected=(err==0);
        ln->loginSent=false;
        // Trois échecs maximum par serveur pendant le cycle courant.
        if (err == 0) {
          ln->lastConnectAttempt = 0;
          ln->connectFailureStreak = 0;
        } else if (ln->connectFailureStreak < 3) {
          ln->connectFailureStreak++;
          addLog(String("TCP SRV") + String(sock) + " : ECHEC CIPOPEN " +
                 String(ln->connectFailureStreak) + "/3");
        }
      }
    }
  }

  if (u.startsWith("+CIPCLOSE:")) {
    int sock=l.substring(l.indexOf(':')+1).toInt();
    TcpLinkState *ln=(sock==1?&link1:(sock==2?&link2:nullptr));
    if(ln) {
      ln->connected=false; ln->opening=false; ln->sending=false; ln->loginSent=false;
      // Conserver lastConnectAttempt afin de respecter le délai de retry.
      ln->reconnectCount++;

      // Déconnexion d'un socket : ne pas fermer le service IP global.
      // taskTCP relancera uniquement le socket concerné après le délai prévu.
      addLog(String("TCP SRV")+String(sock)+" DISCONNECTED -> RETRY");
    }
  }

  // Réception TCP A76XX : +RECEIVE,<link num>,<data length>
  // Le Header-Type=1 de CIPCCFG garantit que le socket source est conservé.
  if (u.startsWith("+RECEIVE,")) {
    int p1 = u.indexOf(',');
    int p2 = u.indexOf(',', p1 + 1);
    if (p1 >= 0 && p2 > p1) {
      int sock = u.substring(p1 + 1, p2).toInt();
      int len = u.substring(p2 + 1).toInt();
      if (sock >= 0 && sock < 2 && len > 0 && len <= 2048) {
        modemRxSocket = (uint8_t)sock;
        modemRxExpected = (uint16_t)len;
        modemRxPayload = "";
        addLog(String("TCP RX SRV") + String(sock + 1) + " LEN:" + String(len));
      }
    }
  }

  if (u.indexOf("NO CARRIER")>=0 || u.indexOf("PDP DEACT")>=0) {
    // Perte réseau/DATA : reconstruire NETOPEN puis TCP.
    tcpServiceOpen=false; modemDataReady=false; tcpNetOpening=false; lastNetOpenAttempt=0; modemStage=MODEM_STAGE_DATA;
    link1.connected=false; link1.opening=false; link1.loginSent=false;
    link2.connected=false; link2.opening=false; link2.loginSent=false;
  }

  if (u.indexOf("RDY")>=0) {
    modemATReady=false; modemDataReady=false;
    tcpServiceOpen=false; tcpNetOpening=false;
    modemStage=MODEM_STAGE_SYNC; modemStageSince=millis();
    modemFailureCount=0;
  }
}

void taskNetwork() {
  if (modemOTAActive) return;

  if (!modemPower) {
    modemState="OFF";
    networkRegistered=false;
    packetServiceAttached=false;
    tcpServiceOpen=false;
    modemDataReady=false;
    return;
  }

  static unsigned long lastInfoPoll=0;
  const unsigned long now=millis();

  // Verification periodique du niveau radio.
  if (modemStage >= MODEM_STAGE_SIGNAL && !modemTcpBusy &&
      !modemWaitingResponse && now-lastInfoPoll >= 30000UL) {
    lastInfoPoll=now;
    modemSendAT("AT+CSQ");
  }

  // Regle de coherence centrale : sans SIM ou sans enregistrement
  // reseau (Home=1 / Roaming=5), il n'existe jamais d'acces Internet
  // valide, meme si une ancienne reponse CGATT indiquait 1.
  if (!simReady || !networkRegistered) {
    packetServiceAttached=false;
    tcpServiceOpen=false;
    tcpNetOpening=false;
    modemDataReady=false;
    link1.connected=false; link1.opening=false; link1.sending=false; link1.loginSent=false;
    link2.connected=false; link2.opening=false; link2.sending=false; link2.loginSent=false;
  }

  // Si le modem est pret, on recontrole aussi la SIM periodiquement.
  // Cela permet de detecter une extraction/insertion physique sans
  // attendre obligatoirement le redemarrage periodique du mode TEST.
  if (modemStage == MODEM_STAGE_READY && !modemTcpBusy &&
      !modemWaitingResponse && now-lastSimPoll >= SIM_STATUS_POLL_INTERVAL_MS) {
    lastSimPoll=now;
    modemSendAT("AT+CPIN?");
  }

  // CGATT n'est plus interrogé ni utilisé comme verrou de connexion.
  // Le modem gère l'attachement PDP pendant NETOPEN.
  if (!tcpServiceOpen) modemDataReady=false;
}

// =========================
// SERVER / GPS103 TCP MANAGER
// =========================
void initLinks() {
  // Allocation modem : MQTT = client 0 ; TCP Serveur 1 = socket 1 ; TCP Serveur 2 = socket 2.
  // Les trois canaux sont ainsi strictement separes.
  link1.enabled = cfg.server1Enable && cfg.server1.length() && cfg.port1;
  link1.host = cfg.server1; link1.port = cfg.port1; link1.socket = 1;
  link2.enabled = cfg.server2Enable && cfg.server2.length() && cfg.port2;
  link2.host = cfg.server2; link2.port = cfg.port2; link2.socket = 2;
}

TcpLinkState* getLink(uint8_t socket) {
  if (socket == 1) return &link1;
  if (socket == 2) return &link2;
  return nullptr;
}

uint8_t tcpSocketMask(uint8_t socket) {
  if (socket == 1) return 0x01;
  if (socket == 2) return 0x02;
  return 0;
}

// ----------------------------------------------------------------
// Autorisation d'essai de connexion serveur (CIPOPEN).
// Séquence volontairement stricte, basée sur la logique historique :
//   SIM READY + réseau enregistré/roaming
//        -> AT+CGDCONT=1,"IP","APN"
//        -> AT+NETOPEN
//        -> +NETOPEN: 0
//        -> AT+CIPOPEN=socket,"TCP",server,port
//        -> +CIPOPEN: socket,0
// Aucun CIPOPEN n'est lancé avant l'ouverture réelle du service IP.
// ----------------------------------------------------------------
bool tcpAttemptAllowed() {
  // Equivalent de la logique historique :
  // if (SIMSTAT=="READY" && (network=="Rooaming" || network=="Registred"))
  // puis CGDCONT -> NETOPEN -> CIPOPEN.
  // Ici SIMSTAT/network sont derives des reponses reelles du modem :
  //   +CPIN: READY
  //   +CEREG/+CGREG: 1 ou 5
  // et le service IP doit etre reellement ouvert avant CIPOPEN.
  return modemPower && simReady && networkRegistered && tcpServiceOpen;
}

void sendCIPOpen(TcpLinkState &ln) {
  // Maximum 3 tentatives CIPOPEN par serveur pendant un cycle de communication.
  if (ln.connectFailureStreak >= 3) return;
  if (!tcpAttemptAllowed() || !ln.enabled || ln.opening || ln.connected) return;
  if (modemTcpBusy || modemExpectSendPrompt || modemWaitingResponse || smsTxBusy) return;
  if (ln.lastConnectAttempt != 0 &&
      millis()-ln.lastConnectAttempt < TCP_RECONNECT_INTERVAL_MS) return;
  ln.lastConnectAttempt = millis();
  ln.opening = true;
  ln.openingSince = millis();

  String cmd = "AT+CIPOPEN=" + String(ln.socket) +
               ",\"TCP\",\"" + ln.host + "\"," + String(ln.port);

  modemSendAT(cmd);
}

void sendCIPData(TcpLinkState &ln, const String &data, bool ignoreConnectedState) {
  if (!modemPower || !ln.connected || ln.sending || modemTcpBusy ||
      modemExpectSendPrompt || smsTxBusy || data.length() == 0) return;

  if (data.length() > 1400 || modemWaitingResponse) return;

  ln.sending = true;
  modemTcpPending = data;
  modemTcpPendingSocket = ln.socket;
  modemExpectSendPrompt = true;
  modemTcpBusy = true;
  modemTcpStartedAt = millis();
  modemTcpDataSentAt = 0;
  modemSendAT("AT+CIPSEND=" + String(ln.socket));
}

// ----------------------------------------------------------------
// Recuperation TCP simple : NETCLOSE -> NETOPEN -> CIPOPEN -> SEND.
// La trame FIFO reste en memoire tant que CIPSEND n'est pas confirme.
// ----------------------------------------------------------------
void handleServerReply(const String &line) {
  String rx = line;
  rx.trim();

  // Confirmation modem : +CIPSEND: socket,sent,confirmed.
  // Attention : ce n'est pas un ACK applicatif emis par le serveur.
  // La suppression reste conditionnee a la confirmation de tous les sockets requis.
  if (rx.startsWith("+CIPSEND:")) {
    String v = rx.substring(rx.indexOf(':') + 1);
    v.trim();
    int p1 = v.indexOf(',');
    int p2 = (p1 >= 0) ? v.indexOf(',', p1 + 1) : -1;

    if (p1 > 0 && p2 > p1) {
      uint8_t socket = (uint8_t)v.substring(0, p1).toInt();
      long sent = v.substring(p1 + 1, p2).toInt();
      long confirmed = v.substring(p2 + 1).toInt();

      if (socket >= 1 && socket <= 2 && modemTcpPendingSocket == socket &&
          sent > 0 && sent == confirmed &&
          sent == (long)modemTcpPending.length()) {

        TcpLinkState &ln = *getLink(socket);
        ln.sending = false;
        ln.connected = true;
        ln.loginSent = (modemTxPurpose == 0) ? true : ln.loginSent;
        ln.lastSend = millis();
        ln.lastTxSuccessAt = millis();

        if (modemTxPurpose == 2) {
          bufferConfirmedMask |= tcpSocketMask(socket);

          if ((bufferConfirmedMask & bufferTargetMask) == bufferTargetMask) {
            if (removeOldestBufferFrame()) {
              recoveryFrame = "";
              bufferTargetMask = 0;
              bufferConfirmedMask = 0;
              bufferActiveSocket = 255;
            }
          }
        } else if (modemTxPurpose == 3) {
          checkPositionConfirmedMask |= tcpSocketMask(socket);
          if ((checkPositionConfirmedMask & checkPositionTargetMask) == checkPositionTargetMask) {
            checkPositionFrame = "";
            checkPositionTargetMask = 0;
            checkPositionConfirmedMask = 0;
            lastCheckPositionSend = millis();
          } else {
            // Si deux serveurs sont actifs, envoyer la meme position au second
            // serveur avant de considerer le cycle CHECK termine.
            if ((checkPositionTargetMask & 0x01) && !(checkPositionConfirmedMask & 0x01) && link1.connected) {
              sendCIPData(link1, checkPositionFrame, false);
              if (link1.sending) modemTxPurpose = 3;
            } else if ((checkPositionTargetMask & 0x02) && !(checkPositionConfirmedMask & 0x02) && link2.connected) {
              sendCIPData(link2, checkPositionFrame, false);
              if (link2.sending) modemTxPurpose = 3;
            }
          }
        }

        modemTcpBusy = false;
        modemExpectSendPrompt = false;
        modemTcpStartedAt = 0;
        modemTcpDataSentAt = 0;
        modemTcpPendingSocket = 255;
        modemTcpPending = "";
        modemTxPurpose = 0;
        bufferActiveSocket = 255;
        return;
      }
    }

    // Une confirmation incomplète signifie que la trame n'est pas
    // considérée comme envoyée. Le serveur sera reconnecté.
    if (modemTcpPendingSocket >= 1 && modemTcpPendingSocket <= 2) {
      TcpLinkState &ln = *getLink(modemTcpPendingSocket);
      ln.sending = false;
      ln.connected = false;
      ln.loginSent = false;
    }
    modemTcpBusy = false;
    modemExpectSendPrompt = false;
    modemTcpStartedAt = 0;
    modemTcpDataSentAt = 0;
    modemTcpPendingSocket = 255;
    modemTcpPending = "";
    modemTxPurpose = 0;
    bufferActiveSocket = 255;
    return;
  }

  // Erreur pendant l'envoi : la trame reste dans le buffer.
  String u = rx;
  u.toUpperCase();
  if (u == "ERROR" || u.indexOf("CIPERROR") >= 0 ||
      u.indexOf("SEND FAIL") >= 0) {

    if (modemTcpPendingSocket >= 1 && modemTcpPendingSocket <= 2) {
      TcpLinkState &ln = *getLink(modemTcpPendingSocket);
      ln.sending = false;
      ln.connected = false;
      ln.loginSent = false;
    }

    modemTcpBusy = false;
    modemExpectSendPrompt = false;
    modemTcpStartedAt = 0;
    modemTcpDataSentAt = 0;
    modemTcpPendingSocket = 255;
    modemTcpPending = "";
    modemTxPurpose = 0;
    bufferActiveSocket = 255;
  }
}

// ============================================================
// COMMANDES SERVEUR
// ============================================================
// Formats acceptés depuis serveur 1 ou serveur 2 :
//   STATUS
//   CMD:STATUS
//   CMD_STATUS
// Le contenu après CMD: / CMD_ reprend exactement la syntaxe SMS.
// Les commandes serveur sont utiles pour la supervision distante et la configuration.
// Elles utilisent le préfixe CMD: / CMD_ et renvoient le résultat par SMS à un administrateur.
bool isServerCommandPayload(const String &payload) {
  String p = payload;
  p.trim();
  String u = p;
  u.toUpperCase();

  // Format TCP serveur : **,imei:<IMEI>,<COMMANDE>
  // Exemple : **,imei:867255071226940,STATUS
  // Le prefixe et les separateurs sont verifies ici, puis
  // processServerCommandPayload() controle l'IMEI avant execution.
  if (u.startsWith("**,IMEI:")) {
    int comma = p.indexOf(',', 8);
    if (comma > 8 && comma < (int)p.length() - 1) {
      String rxImei = p.substring(8, comma);
      rxImei.trim();
      if (rxImei.length() == 15) {
        bool digitsOnly = true;
        for (uint8_t i = 0; i < rxImei.length(); i++) {
          if (!isDigit(rxImei[i])) { digitsOnly = false; break; }
        }
        if (digitsOnly) return true;
      }
    }
    return false;
  }

  // Formats existants : CMD:STATUS / CMD:LOCATION ...
  if (u.startsWith("CMD:") || u.startsWith("CMD_")) return true;

  // Compatibilite avec les serveurs qui envoient directement la commande.
  if (u == "STATUS" || u == "INFO" || u == "LOCATION" || u == "GPS" ||
      u == "GPS ON" || u == "GPS OFF" || u == "GPS STATUS" || u == "GPS?" ||
      u == "MODEM ON" || u == "MODEM OFF" || u == "MODEM STATUS" || u == "MODEM?" ||
      u == "SMS ON" || u == "SMS OFF" || u == "SMS STATUS" || u == "SMS?" ||
      u == "SERVER1 ON" || u == "SERVER1 OFF" || u == "SERVER1 STATUS" ||
      u == "SERVER2 ON" || u == "SERVER2 OFF" || u == "SERVER2 STATUS" ||
      u == "MQTT ON" || u == "MQTT OFF" || u == "MQTT STATUS" || u == "MQTT?" ||
      u == "BATTERY" || u == "BAT" || u == "HELP" || u == "AIDE" ||
      u == "COMMANDES" || u.startsWith("MODE_") ||
      u.startsWith("ADMIN1 ") || u.startsWith("ADMIN2 ") || u.startsWith("ADMIN3 ") ||
      u == "ADMINS?" || u.startsWith("MQTTREPLY ") || u == "MQTTREPLY?" ||
      u.startsWith("GPSINT_") || u.startsWith("SENDINT_") ||
      u.startsWith("APN_") ||
      u.startsWith("SERVER1_") || u.startsWith("SERVER2_") ||
      u == "SAVE" || u == "RESTART" || u == "FACTORY" ||
      u == "FACTORYRESET" || u == "RESETFACTORY" ||
      u.startsWith("UPDATE_")) return true;

  return false;
}

String serverCommandFeedbackNumber() {
  if (cfg.smsAdmin1.length()) return cfg.smsAdmin1;
  if (cfg.smsAdmin2.length()) return cfg.smsAdmin2;
  if (cfg.smsAdmin3.length()) return cfg.smsAdmin3;
  return "";
}

String mqttCommandFeedbackNumber() {
  if (validPhoneNumber(cfg.mqttReplyNumber)) return cfg.mqttReplyNumber;
  return serverCommandFeedbackNumber();
}

void processServerCommandPayload(const String &payload) {
  String p = payload;
  p.trim();
  if (!isServerCommandPayload(p)) return;

  // Format TCP principal : **,imei:<IMEI>,<COMMANDE>
  // Exemple : **,imei:867255071226940,STATUS
  // L'IMEI du serveur doit correspondre a celui du tracker.
  String u = p;
  u.toUpperCase();
  if (u.startsWith("**,IMEI:")) {
    int comma = p.indexOf(',', 8);
    if (comma <= 8 || comma >= (int)p.length() - 1) return;

    String rxImei = p.substring(8, comma);
    rxImei.trim();

    String localImei = cfg.imei;
    if (localImei.length() != 15) localImei = cfg.deviceID;
    localImei.trim();

    if (rxImei != localImei) {
      traceModemRX("[TCP CMD] IMEI INVALIDE: " + rxImei);
      return;
    }

    p = p.substring(comma + 1);
    p.trim();
    if (!p.length()) return;

    traceModemRX("[TCP CMD] COMMANDE: " + p);
  }

  if (p.startsWith("CMD:")) p = p.substring(4);
  else if (p.startsWith("CMD_")) p = p.substring(4);
  p.trim();
  if (!p.length()) return;

  String previousSender = smsSender;
  smsSender = serverCommandFeedbackNumber();
  remoteCommandContext = true;

  // Le serveur TCP est deja authentifie : on reutilise le moteur SMS.
  // Le mot de passe local est ajoute aux commandes classiques.
  if (!smsSender.length() || !cfg.smsPassword.length()) {
    remoteCommandContext = false;
    smsSender = previousSender;
    return;
  }

  String up = p;
  up.toUpperCase();
  if (up.startsWith("UPDATE_")) {
    handleUpdateSMSCommand(p);
  } else {
    handleSMSCommand(p + "_" + cfg.smsPassword);
  }
  remoteCommandContext = false;
  smsSender = previousSender;
}

void processGPS103ServerLogic(const String &line) {
  String u=line;
  u.toUpperCase();
  // Format A7670E courant : +RECEIVE,<socket>,<length>
  if (u.startsWith("+RECEIVE,")) {
    int p1=u.indexOf(',');
    int p2=u.indexOf(',', p1+1);
    if (p1>=0) {
      int sock=u.substring(p1+1, p2>p1?p2:u.length()).toInt();
      if(sock>=0 && sock<2) modemRxSocket=(uint8_t)sock;
    }
  }
  handleServerReply(line);
}

// ----------------------------------------------------------------
// GARDE-FOU "CONNEXION FANTOME"
// Le flag ln.connected n'est mis à jour QUE par les URC du modem
// (+CIPOPEN, +CIPCLOSE, NETCLOSE, NO CARRIER, PDP DEACT, erreurs
// CIPSEND). Si un de ces URC est manqué par le modem A7670E
// (coupure réseau silencieuse côté opérateur, session NAT expirée,
// UART chargé, etc.), ln.connected peut rester bloqué à "true"
// indéfiniment alors que le lien est en réalité mort : c'est la
// cause typique du "déconnecté non détecté".
// On corrige cela en vérifiant qu'au moins une trame est
// confirmée (CIPSEND OK) dans un délai raisonnable après la
// dernière transmission réussie (login, heartbeat ou data).
// Si ce délai est dépassé, on force une reconnexion complète.
// ----------------------------------------------------------------
void taskTCP() {
  if (modemOTAActive) return;
  // ---------------------------------------------------------------
  // 1. Conditions minimales pour le TCP
  // ---------------------------------------------------------------
  // Seules conditions de départ : modem actif, SIM prête et réseau enregistré.
  // NETOPEN doit ensuite confirmer le service IP avant CIPOPEN.
  if (!modemPower || !simReady || !networkRegistered || !tcpServiceOpen) return;

  // Mettre les paramètres des serveurs à jour depuis la configuration.
  initLinks();

  // ---------------------------------------------------------------
  // 2. CONNEXION SERVEUR
  // ---------------------------------------------------------------
  // Si le modem ne répond pas à CIPOPEN, ne pas rester bloqué
  // indéfiniment en état "opening" : on autorise une nouvelle tentative.
  unsigned long now = millis();
  if (link1.opening && now - link1.openingSince >= TCP_CIPOPEN_TIMEOUT_MS) {
    link1.opening = false;
    link1.openingSince = 0;
    if (link1.connectFailureStreak < 3) link1.connectFailureStreak++;
    addLog("TCP SRV1 : TIMEOUT CIPOPEN " + String(link1.connectFailureStreak) + "/3");
  }
  if (link2.opening && now - link2.openingSince >= TCP_CIPOPEN_TIMEOUT_MS) {
    link2.opening = false;
    link2.openingSince = 0;
    if (link2.connectFailureStreak < 3) link2.connectFailureStreak++;
    addLog("TCP SRV2 : TIMEOUT CIPOPEN " + String(link2.connectFailureStreak) + "/3");
  }

  // Tant qu'un serveur n'est pas connecté, on tente CIPOPEN.
  // Aucune trame n'est envoyée avant la confirmation +CIPOPEN: socket,0.
  if (link1.enabled && !link1.connected) {
    sendCIPOpen(link1);
    if (link1.opening || modemWaitingResponse) return;
  }

  if (link2.enabled && !link2.connected) {
    sendCIPOpen(link2);
    if (link2.opening || modemWaitingResponse) return;
  }

  // ---------------------------------------------------------------
  // 3. Une transmission à la fois
  // ---------------------------------------------------------------
  if (modemTcpBusy || modemExpectSendPrompt || modemWaitingResponse || smsTxBusy)
    return;

  // ---------------------------------------------------------------
  // 4. BUFFER : priorité absolue
  // ---------------------------------------------------------------
  if (bufferPending > 0) {

    if (recoveryFrame.length() == 0) {
      if (!peekBufferFrame(recoveryFrame)) return;

      bufferTargetMask = 0;
      bufferConfirmedMask = 0;
      bufferActiveSocket = 255;

      if (link1.enabled) bufferTargetMask |= 0x01;
      if (link2.enabled) bufferTargetMask |= 0x02;
    }

    if (bufferTargetMask == 0) return;

    // Si tous les sockets requis ont deja confirme CIPSEND mais que
    // SPIFFS a refuse le truncate, retenter la finalisation sans renvoyer
    // la trame et sans la supprimer avant reussite du truncate.
    if ((bufferConfirmedMask & bufferTargetMask) == bufferTargetMask) {
      if (removeOldestBufferFrame()) {
        recoveryFrame = "";
        bufferTargetMask = 0;
        bufferConfirmedMask = 0;
        bufferActiveSocket = 255;
      }
      return;
    }

    // Serveur 1 en premier.
    if ((bufferTargetMask & 0x01) &&
        !(bufferConfirmedMask & 0x01) && link1.connected) {
      sendCIPData(link1, recoveryFrame, false);
      if (link1.sending) {
        bufferActiveSocket = 0;
        modemTxPurpose = 2;
      }
      return;
    }

    // Puis serveur 2 si activé.
    if ((bufferTargetMask & 0x02) &&
        !(bufferConfirmedMask & 0x02) && link2.connected) {
      sendCIPData(link2, recoveryFrame, false);
      if (link2.sending) {
        bufferActiveSocket = 1;
        modemTxPurpose = 2;
      }
      return;
    }

    // Un serveur requis n'est pas connecté : attendre sa reconnexion.
    return;
  }

  // ---------------------------------------------------------------
  // 5. LOGIN / HEARTBEAT
  // ---------------------------------------------------------------
  if (link1.enabled && link1.connected && !link1.sending) {
    if (!link1.loginSent) {
      String login = "##,imei:" + cfg.imei + ",A;";
      sendCIPData(link1, login, false);
      if (link1.sending) modemTxPurpose = 0;
      return;
    }

    uint16_t hb = (cfg.heartbeatInterval < 1) ? 1 : cfg.heartbeatInterval;
    if (millis() - link1.lastHeartbeat >= (unsigned long)hb * 1000UL) {
      sendCIPData(link1, buildGPS103Heartbeat(), false);
      if (link1.sending) {
        link1.lastHeartbeat = millis();
        modemTxPurpose = 1;
      }
    }
  }
}

// ============================================================
// SUPERVISION DE SECURITE DES SESSIONS D'ENVOI
// Couche supplementaire : la production des trames, le buffer et
// la logique des modes restent inchanges.
// ============================================================
void taskSendRecovery() {
  if (modemOTAActive || !modemPower || scheduledCommunicationWake ||
      (bufferPending == 0 && recoveryFrame.length() == 0 &&
       checkPositionFrame.length() == 0)) {
    lastRecovery = 0;
    return;
  }

  uint8_t required = 0;
  if (link1.enabled) required |= 0x01;
  if (link2.enabled) required |= 0x02;
  if (!required) {
    lastRecovery = 0;
    return;
  }

 const bool communicationOK =
    simReady && networkRegistered &&
    modemDataReady && tcpServiceOpen &&
    ((connectedServerMask() & required) == required);

  // Le maintien de communication est le budget GLOBAL du cycle de connexion.
  // Une connexion rétablie remet le compteur à zéro.
  if (communicationOK) {
    lastRecovery = 0;
    return;
  }

  const unsigned long now = millis();
  if (lastRecovery == 0) {
    lastRecovery = now;
    return;
  }

  const unsigned long maintenanceMs = (unsigned long)max((uint16_t)1, cfg.heartbeatInterval) * 1000UL;
  if (now - lastRecovery < maintenanceMs) return;

  // Fin du budget de communication : aucune trame n'est supprimée.
  lastRecovery = 0;
  modemRecoverySince = now;
  modemStage = MODEM_STAGE_RECOVERY;
  addLog("CONNEXION ECHEC : MAINTIEN COMMUNICATION EXPIRE");

  // NORMAL / ECO / ULTRA ECO : sommeil immédiat puis nouvel essai au réveil.
  if (cfg.energyMode == 0 || cfg.energyMode == 1 || cfg.energyMode == 3) {
    setModemPower(false);
    setGPSPower(false);
    enterConfiguredDeepSleep("CONNEXION ECHEC / PROCHAIN REVEIL");
    return;
  }

  // PERFORMANCE / PERMANENCE / TEST : récupération sans Deep Sleep imposé.
  // Réarmer les trois tentatives pour le cycle de reconnexion suivant.
  link1.connectFailureStreak = 0;
  link2.connectFailureStreak = 0;
  addLog("RECUPERATION : MODE ACTIF / NOUVEAU CYCLE TCP");
  setModemPower(false);
}

void taskServer() {
  taskTCP();
}

bool anyServerConnected() {
  return (link1.enabled && link1.connected) || (link2.enabled && link2.connected);
}

uint8_t connectedServerMask() {
  uint8_t m=0;
  if (link1.enabled && link1.connected) m |= 0x01;
  if (link2.enabled && link2.connected) m |= 0x02;
  return m;
}

// ------------------------------------------------------------------
// Intervalle de position adaptatif
//
// cfg.positionInterval est TOUJOURS l'intervalle maximal.
// La vitesse ne peut que DIVISER cet intervalle :
// vitesse faible  -> diviseur 1 -> intervalle maximal
// vitesse élevée  -> diviseur plus grand -> intervalle plus court
//
// Les seuils/diviseurs sont centralisés ici pour être faciles à régler.
// ------------------------------------------------------------------
uint8_t positionSpeedDivider(double speedKmh) {
  if (speedKmh < 10.0) return 1;
  if (speedKmh < 30.0) return 2;
  if (speedKmh < 50.0) return 3;
  if (speedKmh < 80.0) return 5;
  return 10;
}

unsigned long effectivePositionIntervalMs() {
  uint32_t baseSeconds = max((uint16_t)1, cfg.positionInterval);
  double speedKmh = max(0.0, gps.speed) * 1.852; // RMC fournit des noeuds
  uint8_t divider = positionSpeedDivider(speedKmh);
  uint32_t effectiveSeconds = baseSeconds / divider;

  // Le diviseur ne doit jamais produire 0 seconde.
  if (effectiveSeconds < 1) effectiveSeconds = 1;
  return (unsigned long)effectiveSeconds * 1000UL;
}

// Variation angulaire minimale entre deux caps, correcte sur 0/360°.
double courseVariation(double a, double b) {
  if (a < 0.0 || b < 0.0) return 0.0;
  double d = fabs(a - b);
  if (d > 180.0) d = 360.0 - d;
  return d;
}

void taskPositionManager() {
  // ================================================================
  // ENREGISTREMENT PERMANENT DES POSITIONS
  // ================================================================
  // NORMAL/ECO : aucune position n'est enregistrée avant la
  // confirmation réelle du mouvement (> 6 km/h).
  // Toute position valide est d'abord écrite dans le buffer.
  // Aucune position ne part directement vers le serveur.
  if (!gps.fix || cfg.imei.length() != 15) return;

  // NORMAL et PERFORMANCE partagent la meme autorisation de capture :
  // mouvement confirme. NORMAL ne depend plus du seul drapeau activeTracking,
  // qui pouvait retarder/perdre des points lors d'une transition de suivi.
  if (cfg.energyMode == 0 && (!moving || stopConfirmed)) return;
  // ECO conserve sa regle energetique existante.
  if (cfg.energyMode == 1 && !activeTracking) return;

  // PERFORMANCE : apres confirmation de l'arret, ne plus creer de
  // nouvelles positions. Les positions deja presentes dans le buffer
  // restent disponibles pour la transmission. Le GPS reste actif.
  if (cfg.energyMode == 2 && stopConfirmed)
    return;

  // PERMANENCE : le GPS reste actif mais l'enregistrement est réservé
  // au déplacement confirmé.
  if (cfg.energyMode == 5 && !moving)
    return;

  unsigned long now = millis();

  // Mode TEST : comportement historique indépendant de l'adaptation.
  if (cfg.energyMode == 4) {
    if (lastPositionEvent == 0 || now - lastTestTx >= 15000UL) {
      lastTestTx = now;
      if (storePosition()) {
        lastPositionEvent = now;
        lastTurnCourse = gps.course;
        addLog("TEST -> POSITION ENREGISTREE BUFFER");
      }
    }
    return;
  }

  // Première position : elle devient la référence de temps et de cap.
  if (lastPositionEvent == 0) {
    if (storePosition()) {
      lastPositionEvent = now;
      lastTurnCourse = gps.course;
      addLog("POSITION -> BUFFER (premiere)");
    }
    return;
  }

  // CONDITION 1 : variation de cap supérieure au seuil configuré.
  // Elle déclenche une trame immédiatement, même si l'intervalle
  // dynamique n'est pas encore arrivé.
  bool turnTrigger = false;
  if (gps.course >= 0.0 && lastTurnCourse >= 0.0) {
    double deltaCourse = courseVariation(gps.course, lastTurnCourse);
    turnTrigger = (deltaCourse > (double)cfg.turnAngle);
  }

  // CONDITION 2 : intervalle dynamique atteint.
  // positionInterval reste la valeur maximale ; la vitesse ne fait
  // qu'appliquer un diviseur.
  unsigned long intervalMs = effectivePositionIntervalMs();
  bool intervalTrigger = (now - lastPositionEvent >= intervalMs);

  if (turnTrigger || intervalTrigger) {
    if (storePosition()) {
      lastPositionEvent = now;
      if (gps.course >= 0.0) lastTurnCourse = gps.course;
      if (turnTrigger) addLog("POSITION -> BUFFER (VARIATION CAP)");
      else addLog("POSITION -> BUFFER (INTERVALLE VITESSE)");
    }
  }
}


// ============================================================
// VOYANT DE SIGNALISATION - SORTIE 2
// ============================================================
void taskStatusLED() {
  if (!cfg.ledEnable) {
    digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF_LEVEL);
    return;
  }

  const unsigned long now = millis();
  bool on = false;
  unsigned long period = 0;

  if (scheduledCommunicationWake) {
    // Communication périodique : clignotement rapide.
    period = scheduledGrandWake ? 250UL : 500UL;
    on = ((now / period) % 2UL) == 0;
  } else if (wakeVibrationWindow) {
    // Surveillance vibration : clignotement lent.
    period = 700UL;
    on = ((now / period) % 2UL) == 0;
  } else if (motionConfirmationActive) {
    // Confirmation de mouvement : clignotement intermédiaire.
    period = 350UL;
    on = ((now / period) % 2UL) == 0;
  } else if (moving) {
    // Mouvement confirmé : voyant fixe.
    on = true;
  } else if (modemPower) {
    // Communication active sans mouvement : clignotement lent.
    period = 1000UL;
    on = ((now / period) % 2UL) == 0;
  } else if (batteryProtectionActive) {
    period = 180UL;
    on = ((now / period) % 2UL) == 0;
  }

  digitalWrite(STATUS_LED_PIN, on ? STATUS_LED_ON_LEVEL : STATUS_LED_OFF_LEVEL);
}

// =========================
// MOVEMENT MANAGER
// =========================
float courseDifference(double a, double b) {
  float d = fabs(a - b);
  while (d >= 360.0f) d -= 360.0f;
  if (d > 180.0f) d = 360.0f - d;
  return d;
}

// ============================================================
// RÉVEIL COMMUNICATION PÉRIODIQUE
// ============================================================
void taskScheduledCommunication() {
  const unsigned long now = millis();

  // PERMANENCE : réveil périodique basé sur millis(), car le mode ne passe
  // jamais par Deep-Sleep et le compteur RTC ne progresse donc pas.
  if (cfg.energyMode == 5 && !scheduledCommunicationWake && cfg.longIntervalMin > 0) {
    const unsigned long period = (unsigned long)cfg.longIntervalMin * 60000UL;
    if (permanenceNextWakeAt == 0) permanenceNextWakeAt = now + period;
    if ((long)(now - permanenceNextWakeAt) >= 0) {
      scheduledCommunicationWake = true;
      scheduledGrandWake = true;
      communicationWakeStartedAt = 0;
      communicationWakeWindowMs = (unsigned long)max((uint16_t)1, cfg.heartbeatInterval) * 1000UL;
      permanenceNextWakeAt += period;
      if ((long)(now - permanenceNextWakeAt) >= 0) permanenceNextWakeAt = now + period;
      checkPositionFrame = "";
      checkPositionTargetMask = 0;
      checkPositionConfirmedMask = 0;
    }
  }

  if (!scheduledCommunicationWake) return;

  // PERMANENCE : le REVEIL est un contrôle périodique. Le modem est déjà ON.
  // A l'arrêt, aucune position automatique n'est créée, mais SMS/MQTT restent
  // disponibles en permanence.
  if (cfg.energyMode == 5 && !moving) {
    setGPSPower(true);
    if (!modemPower) setModemPower(true);
    if (permanenceWakeStartedAt == 0) permanenceWakeStartedAt = now;
    if (gps.fix && gps.speed > MOTION_SPEED_THRESHOLD_KMH) {
      moving = true;
      stopConfirmed = false;
      lastMovement = now;
      activeTracking = true;
      scheduledCommunicationWake = false;
      scheduledGrandWake = false;
      permanenceWakeStartedAt = 0;
      return;
    }

    
if (now - permanenceWakeStartedAt >= 1500UL &&
    communicationWakeStartedAt == 0) {

    // LONGINT en PERMANENCE :
    // envoyer une trame de présence même si le véhicule est arrêté.
    scheduledGrandWake = true;
    communicationWakeStartedAt = now;
    communicationWakeWindowMs = (unsigned long)max((uint16_t)1, cfg.heartbeatInterval) * 1000UL;

    // Force une émission immédiate.
    lastCheckPositionSend = now - CHECK_POSITION_INTERVAL_MS;

    checkPositionFrame = "";
    checkPositionTargetMask = 0;
    checkPositionConfirmedMask = 0;

    permanenceModemSleep = false;
    setModemPower(true);
}

// Attendre la fin des 1,5 s avant de continuer.
if (permanenceWakeStartedAt == 0 ||
    now - permanenceWakeStartedAt < 1500UL) {
    return;
}
}
  if (!communicationWakeStartedAt) {
    communicationWakeStartedAt = now;
    // Chaque réveil programmé dispose de ses propres trois tentatives TCP.
    link1.connectFailureStreak = 0;
    link2.connectFailureStreak = 0;
    lastRecovery = 0;
    lastCheckPositionSend = now - CHECK_POSITION_INTERVAL_MS;
    checkPositionFrame = "";
    checkPositionTargetMask = 0;
    checkPositionConfirmedMask = 0;
  }

  // CHECK : GPS + communication sont disponibles pendant la fenetre.
  // Les commandes SMS et distantes ne sont recherchees que pendant CHECK.
  setGPSPower(true);
  setModemPower(true);

  // Envoi d'une position FIX toutes les 30 s vers les serveurs TCP actifs.
  // La position n'est pas retiree du buffer et ne modifie pas le store-forward.
  if (gps.fix && !modemTcpBusy && !modemExpectSendPrompt &&
      !modemWaitingResponse && !smsTxBusy) {
    initLinks();

    // Finir d'abord l'envoi de la position courante au serveur restant.
    if (checkPositionFrame.length() > 0) {
      if ((checkPositionTargetMask & 0x01) && !(checkPositionConfirmedMask & 0x01) && link1.connected) {
        sendCIPData(link1, checkPositionFrame, false);
        if (link1.sending) { modemTxPurpose = 3; return; }
      }
      if ((checkPositionTargetMask & 0x02) && !(checkPositionConfirmedMask & 0x02) && link2.connected) {
        sendCIPData(link2, checkPositionFrame, false);
        if (link2.sending) { modemTxPurpose = 3; return; }
      }
    }

    if (checkPositionFrame.length() == 0 &&
        now - lastCheckPositionSend >= CHECK_POSITION_INTERVAL_MS) {
      checkPositionTargetMask = 0;
      checkPositionConfirmedMask = 0;
      if (link1.enabled && link1.connected) checkPositionTargetMask |= 0x01;
      if (link2.enabled && link2.connected) checkPositionTargetMask |= 0x02;

      if (checkPositionTargetMask) {
        checkPositionFrame = buildGPS103Frame();
        if (!checkPositionFrame.length()) {
          checkPositionTargetMask = 0;
          checkPositionConfirmedMask = 0;
          lastCheckPositionSend = now;
          return;
        }
        if ((checkPositionTargetMask & 0x01) && link1.connected) {
          sendCIPData(link1, checkPositionFrame, false);
          if (link1.sending) {
            modemTxPurpose = 3;
            return;
          }
        }
        if ((checkPositionTargetMask & 0x02) && link2.connected) {
          sendCIPData(link2, checkPositionFrame, false);
          if (link2.sending) {
            modemTxPurpose = 3;
            return;
          }
        }
        // Aucun envoi n'a pu demarrer : on retentera au prochain passage.
        checkPositionFrame = "";
        checkPositionTargetMask = 0;
        checkPositionConfirmedMask = 0;
      }
    }
  }

  if (now - communicationWakeStartedAt < communicationWakeWindowMs)
    return;


  // Ne pas couper le modem pendant une transmission ou
  // tant qu'une réponse SMS est en attente.
  if (modemTcpBusy ||
      modemExpectSendPrompt ||
      smsTxBusy ||
      smsQueueCount > 0 ||
      modemOTAActive) {
    communicationWakeStartedAt = millis();
    return;
  }

  // Fin du CHECK : toutes les communications s'arretent et le modem est OFF.
  if (moving || motionConfirmationActive) {
    scheduledCommunicationWake = false;
    scheduledGrandWake = false;
    scheduledSmsWake = false;
    communicationWakeStartedAt = 0;
    communicationWakeWindowMs = 0;
    checkPositionFrame = "";
    checkPositionTargetMask = 0;
    checkPositionConfirmedMask = 0;
    return;
  }

  scheduledCommunicationWake = false;
  scheduledGrandWake = false;
  scheduledSmsWake = false;
  communicationWakeStartedAt = 0;
  communicationWakeWindowMs = 0;
  checkPositionFrame = "";
  checkPositionTargetMask = 0;
  checkPositionConfirmedMask = 0;
  if (cfg.energyMode == 5) {
    setGPSPower(true);
    setModemPower(true);
    // Le modem reste disponible en permanence pour les commandes SMS/MQTT.
    permanenceModemSleep = false;
    return;
  }

  setModemPower(false);
  setGPSPower(false);
  enterConfiguredDeepSleep("FIN CHECK");
}

// ============================================================
// DEMANDE LOCALISATION EN ATTENTE
// ============================================================
void savePendingLocationRequest(const String &number, bool viaMQTT) {
  if (!number.length()) return;

  // Format: MQTT|numero ou SMS|numero.
  // Un ancien fichier contenant uniquement le numero reste compatible.
  File f = SPIFFS.open("/location.pending", "w");
  if (f) {
    f.print(viaMQTT ? "MQTT|" : "SMS|");
    f.print(number);
    f.close();
  } else {
    addLog("LOCATION : ENREGISTREMENT IMPOSSIBLE");
  }

  // Même si l'écriture SPIFFS échoue, conserver la demande en RAM afin
  // qu'elle ne soit pas perdue pendant le cycle courant.
  pendingLocationNumber = number;
  pendingLocationViaMQTT = viaMQTT;
  pendingLocationSMS = true;
  pendingLocationSending = false;
  pendingLocationLastAttempt = 0;
  addLog(String("LOCATION : DEMANDE ENREGISTREE / ") + (viaMQTT ? "SERVEUR CMD" : "SMS"));
}


void loadPendingLocationRequest() {
  pendingLocationSMS = false;
  pendingLocationNumber = "";
  pendingLocationViaMQTT = false;
  if (!SPIFFS.exists("/location.pending")) return;
  File f = SPIFFS.open("/location.pending", "r");
  if (!f) return;
  String saved = f.readString();
  saved.trim();
  f.close();

  if (saved.startsWith("MQTT|")) {
    pendingLocationViaMQTT = true;
    pendingLocationNumber = saved.substring(5);
  } else if (saved.startsWith("SMS|")) {
    pendingLocationNumber = saved.substring(4);
  } else {
    // Compatibilité avec les anciennes versions : numero seul.
    pendingLocationNumber = saved;
  }
  pendingLocationNumber.trim();
  pendingLocationSMS = pendingLocationNumber.length() > 0;
  if (pendingLocationSMS) {
    addLog(String("LOCATION : DEMANDE RECUPEREE / ") + (pendingLocationViaMQTT ? "SERVEUR CMD" : "SMS"));
  }
}

void clearPendingLocationRequest() {
  SPIFFS.remove("/location.pending");
  pendingLocationSMS = false;
  pendingLocationNumber = "";
  pendingLocationViaMQTT = false;
  pendingLocationLastAttempt = 0;
}

void taskPendingLocationSMS() {
  if (modemOTAActive) return;
  if (!pendingLocationSMS) return;

  // Aucune exception de reveil : une demande LOCATION est traitee
  // uniquement pendant une fenetre CHECK deja planifiee.
  if (!scheduledCommunicationWake) return;

  if (!gps.fix || !modemPower || smsTxBusy || smsQueueCount > 0) return;
  if (pendingLocationSending) return;
  if (millis() - pendingLocationLastAttempt < PENDING_LOCATION_RETRY_MS) return;

  pendingLocationLastAttempt = millis();
  pendingLocationSending = true;

  String response = locationSMS();

  // UNE DEMANDE = UNE REPONSE.
  // Pour une commande distante, la localisation est retournee uniquement
  // sur le canal de commande. On ne relance jamais un SMS a chaque retry.
  if (pendingLocationViaMQTT) {
    bool mqttOK = false;
    if (mqttConnected && mqttSubscribed) {
      mqttOK = mqttPublish(mqttCmdTopic(), response, 1, true);
      if (mqttOK) {
        mqttLastAckText = response;
        mqttLastAck = millis();
      }
    }

    // Si la connexion distante est disponible, la demande est terminee
    // definitivement apres cette tentative unique. En cas de perte reseau,
    // on conserve la demande sans envoyer de SMS repetitifs.
    if (mqttOK) {
      clearPendingLocationRequest();
    }
    pendingLocationSending = false;
    return;
  }

  // Demande provenant d'un SMS : une seule transmission SMS lorsque le FIX
  // est disponible. La demande est retiree immediatement de la file afin
  // d'empêcher toute repetition pendant les CHECK suivants.
  smsSend(pendingLocationNumber, response);
  clearPendingLocationRequest();
  pendingLocationSending = false;
}

void taskMovement() {
  if (modemOTAActive) return;
  const unsigned long now = millis();

  // ============================================================
  // PERMANENCE : logique dédiée
  // ESP + GPS + MODEM restent actifs.
  // L'enregistrement automatique n'est autorisé que pendant le mouvement.
  // Les commandes SMS/MQTT restent donc accessibles à l'arrêt.
  // ============================================================
  if (cfg.energyMode == 5) {
    setGPSPower(true);
    setModemPower(true);
    activeTracking = true;
    wakeVibrationWindow = false;
    motionConfirmationActive = false;

    if (gps.fix && gps.speed > MOTION_SPEED_THRESHOLD_KMH) {
      if (!moving) {
        addLog("PERMANENCE : MOUVEMENT CONFIRME");
        lastTurnCourse = -1;
      }
      moving = true;
      stopConfirmed = false;
      lastMovement = now;
      return;
    }

    if (moving && gps.fix && now - lastMovement >= (unsigned long)cfg.stopDelay * 1000UL) {
      moving = false;
      stopConfirmed = true;
      motionEvent = false;
      addLog("PERMANENCE : ARRET CONFIRME / MODEM ON / COMMANDES DISPONIBLES");
    }

    return;
  }

  // Les réveils périodiques de communication sont gérés séparément.
  if (scheduledCommunicationWake) {
    setGPSPower(true);
    if (cfg.energyMode == 5) wakeModemSoftwareSleep();
    else setModemPower(true);
    if (gps.fix && gps.speed > MOTION_SPEED_THRESHOLD_KMH) {
      // Le mouvement est detecte, mais le cycle periodique continue
      // jusqu'a la fin du maintien de communication configure.
      moving = true;
      stopConfirmed = false;
      lastMovement = now;
      activeTracking = true;
    }
    return;
  }

  // MODE TEST : fonctionnement continu.
  if (cfg.energyMode == 4) {
    wakeVibrationWindow = false;
    motionConfirmationActive = false;
    activeTracking = true;
    moving = false;
    stopConfirmed = false;
    setGPSPower(true);
    setModemPower(true);
    return;
  }

  // Fenêtre de détection vibration au réveil.
  if (wakeVibrationWindow) {
    const uint8_t state = (digitalRead(VIBRATION) == HIGH) ? HIGH : LOW;
    if (state != vibrationLastState &&
        now - vibrationLastEdgeAt >= (unsigned long)cfg.vibrationDebounceMs) {
      vibrationLastState = state;
      vibrationLastEdgeAt = now;
      if (vibrationTransitions < 65535) vibrationTransitions++;
      if (vibrationTransitions >= cfg.vibrationThreshold) {
        wakeVibrationWindow = false;
        motionConfirmationActive = true;
        motionConfirmStartedAt = now;
        motionEvent = true;
        lastVibrationEvent = now;
        setGPSPower(true);
        setModemPower(false);
        addLog(String("VIBRATION VALIDEE ") + String(vibrationTransitions) +
               " transitions -> LOCALISATION ACTIVÉE / CONFIRMATION 2 MIN");
        return;
      }
    }
    if (now - wakeStartedAt >= (unsigned long)cfg.vibrationWatchSeconds * 1000UL) {
      addLog(String("VIBRATION INSUFFISANTE ") + String(vibrationTransitions) +
             "/" + String(cfg.vibrationThreshold) + " -> DEEP-SLEEP");
      enterConfiguredDeepSleep("NO VALID VIBRATION");
      return;
    }
    return;
  }

  // Confirmation du vrai mouvement après vibration.
  if (motionConfirmationActive) {
    setGPSPower(true);
    setModemPower(false);
    if (gps.fix && gps.speed > MOTION_SPEED_THRESHOLD_KMH) {
      motionConfirmationActive = false;
      activeTracking = true;
      moving = true;
      stopConfirmed = false;
      lastMovement = now;
      lastVibrationEvent = now;
      lastTurnCourse = -1;
      addLog("MOUVEMENT CONFIRME > 6 KM/H");
      return;
    }
    if (now - motionConfirmStartedAt >= (unsigned long)cfg.motionConfirmTimeout * 1000UL) {
      motionConfirmationActive = false;
      motionEvent = false;
      moving = false;
      stopConfirmed = true;
      if (cfg.energyMode == 5) {
        setGPSPower(true);
        setModemPower(true);
        permanenceModemSleep = false;
        addLog("FIN RECHERCHE FIX / PAS DE MOUVEMENT -> PERMANENCE");
      } else {
        setGPSPower(false);
        setModemPower(false);
        addLog("FIN RECHERCHE FIX / PAS DE MOUVEMENT -> DEEP-SLEEP");
        enterConfiguredDeepSleep("NO MOVEMENT");
      }
      return;
    }
    return;
  }

  if (!activeTracking) return;
  setGPSPower(true);

  if (gps.fix && gps.speed > MOTION_SPEED_THRESHOLD_KMH) {
    moving = true;
    stopConfirmed = false;
    lastMovement = now;

    // ECO : dès qu'un mouvement reprend pendant une récupération,
    // on coupe immédiatement le modem et on reprend l'enregistrement.
    if (cfg.energyMode == 1 && modemPower) {
      setModemPower(false);
      addLog("ECO : MOUVEMENT -> MODEM OFF / ENREGISTREMENT");
    }

    // NORMAL : une récupération déclenchée par 100 éléments continue
    // même si le véhicule reste en mouvement.
    return;
  }

  if (gps.fix && now - lastMovement >= (unsigned long)cfg.stopDelay * 1000UL) {
    moving = false;
    stopConfirmed = true;
    motionEvent = false;
    addLog("ARRET CONFIRME");
  }

  // Le sommeil après arrêt est décidé par le gestionnaire d'énergie.
  // Ici, aucune coupure de transmission normale n'est forcée.
}

// ============================================================
// TEST : REDEMARRAGE PERIODIQUE DU MODEM
// ============================================================
// Strictement actif en mode TEST.
// Le modem est coupe pendant 2 s puis rallume. La machine d'initialisation
// reprend alors naturellement par BOOT -> SYNC -> CPIN -> RESEAU -> DATA.
// Cela permet de detecter une SIM reinserée sans modifier les autres modes.
void taskTestModemRestart() {
  if (cfg.energyMode != 4) {
    testModemRestarting = false;
    testModemRestartStarted = 0;
    testModemLastRestart = millis();
    return;
  }

  const unsigned long now = millis();

  if (testModemRestarting) {
    // Pendant cette courte phase, aucune autre logique ne doit maintenir
    // le modem sous tension.
    if (modemPower) setModemPower(false);

    if (now - testModemRestartStarted >= TEST_MODEM_RESTART_OFF_MS) {
      testModemRestarting = false;
      testModemLastRestart = now;
      setModemPower(true);
      addLog("TEST : MODEM RESTART -> NOUVEAU CHECK SIM");
    }
    return;
  }

  if (!modemPower) return;

  if (now - testModemLastRestart >= TEST_MODEM_RESTART_INTERVAL_MS) {
    testModemRestarting = true;
    testModemRestartStarted = now;
    setModemPower(false);
    addLog("TEST : MODEM OFF -> REINSERTION SIM / RESTART");
  }
}

void enterModemSoftwareSleep() {
  // PERMANENCE V2.14 : le modem reste toujours disponible pour SMS/MQTT.
  if (cfg.energyMode != 5 || !modemPower || permanenceModemSleep) return;
  return;
  if (modemTcpBusy || modemExpectSendPrompt || modemWaitingResponse || smsTxBusy) return;
  // CSCLK=1 autorise le sommeil automatique du modem sans couper son alimentation.
  Serial.print("AT+CSCLK=1\r");
  modemLastActivity = millis();
  modemLastCommand = "AT+CSCLK=1";
  permanenceModemSleep = true;
  modemWaitingResponse = false;
}

void wakeModemSoftwareSleep() {
  // Conservée pour compatibilité avec les autres chemins de code.
  // En PERMANENCE V2.14, le modem ne passe plus en Software Sleep.
  if (cfg.energyMode != 5 || !modemPower || !permanenceModemSleep) return;
  // Une activité UART réveille le modem; la commande désactive ensuite CSCLK.
  Serial.print("\r");
  delay(2);
  Serial.print("AT+CSCLK=0\r");
  modemLastActivity = millis();
  modemLastCommand = "AT+CSCLK=0";
  permanenceModemSleep = false;
  permanenceWakePending = true;
  permanenceWakeStartedAt = millis();
}

void taskModeManager() {
  static uint8_t lastMode = 255;

  if (lastMode != cfg.energyMode) {
    // Réinitialiser uniquement les états de récupération liés au mode.
    normalBufferRecoveryActive = false;
    normalPostStopRecoveryActive = false;
    normalPostStopRecoveryStart = 0;
    if (lastMode == 5 && cfg.energyMode != 5 && permanenceModemSleep) {
      permanenceModemSleep = false;
      if (modemPower) {
        Serial.print("\r");
        delay(2);
        Serial.print("AT+CSCLK=0\r");
      }
    }
    permanenceModemSleep = false;
    permanenceWakePending = false;
    permanenceNextWakeAt = 0;
    lastMode = cfg.energyMode;
    if (cfg.energyMode == 4) {
      testModemRestarting = false;
      testModemRestartStarted = 0;
      testModemLastRestart = millis();
    } else {
      testModemRestarting = false;
    }
    addLog(String("MODE -> ") + (cfg.energyMode==0?"NORMAL":cfg.energyMode==1?"ECO":cfg.energyMode==2?"PERFORMANCE":cfg.energyMode==3?"ULTRA ECO":cfg.energyMode==4?"TEST":"PERMANENCE"));
  }

  if (cfg.energyMode != 4 && batteryVoltage > 0 && batteryVoltage <= cfg.batteryCriticalV) {
    batteryProtectionActive = true;
    taskBatteryProtection();
    if (cfg.batteryAction == 2) return;
  } else {
    batteryProtectionActive = false;
    taskBatteryProtection();
  }

  // MODE TEST : fonctionnement continu.
  if (cfg.energyMode == 4) {
    wakeVibrationWindow = false;
    motionConfirmationActive = false;
    activeTracking = true;
    setGPSPower(true);
    if (!testModemRestarting) setModemPower(true);
    else setModemPower(false);
    return;
  }

  // ---------------------------------------------------------------
  // PERMANENCE
  // ---------------------------------------------------------------
  // ESP + GPS + MODEM restent actifs en permanence.
  // A l'arrêt, aucune nouvelle position automatique n'est enregistrée,
  // mais le modem reste disponible pour les SMS et MQTT.
  if (cfg.energyMode == 5) {
    setGPSPower(true);
    activeTracking = true;

    // Le modem ne doit JAMAIS être coupé ni passer en Software Sleep
    // dans PERMANENCE : l'accès de contrôle reste disponible à tout moment.
    permanenceModemSleep = false;
    permanenceWakePending = false;
    setModemPower(true);
    return;
  }

  // Fenêtre de communication périodique : priorité absolue pendant la durée minimale.
  if (scheduledCommunicationWake) {
    setGPSPower(true);
    setModemPower(true);
    return;
  }

  // Au réveil, rien ne transmet avant la validation du mouvement.
  if (wakeVibrationWindow || motionConfirmationActive) {
    if (!motionConfirmationActive) {
      setGPSPower(false);
      setModemPower(false);
    } else {
      setGPSPower(true);
      setModemPower(false);
    }
    return;
  }

  if (activeTracking) setGPSPower(true);

  const unsigned long now = millis();
  const bool stopDelayDone = stopConfirmed &&
      (now - lastMovement >= (unsigned long)cfg.stopDelay * 1000UL);

  // ---------------------------------------------------------------
  // NORMAL
  // ---------------------------------------------------------------
  // 1) En mouvement : le modem reste OFF sauf si le buffer atteint 100.
  //    Dans ce cas la récupération démarre et continue jusqu'à 0,
  //    même si le véhicule reste en mouvement.
  // 2) Après arrêt + délai : si buffer > 0, fenêtre de 2 minutes.
  // 3) Buffer à 0 : modem OFF.
  // 4) 2 minutes écoulées avec buffer non vide : modem OFF + sommeil.
  if (cfg.energyMode == 0) {
    if (moving) {
      normalPostStopRecoveryActive = false;
      normalPostStopRecoveryStart = 0;

      if (normalBufferRecoveryActive) {
        if (bufferPending == 0) {
          // Ne jamais couper le modem au milieu d'un envoi TCP, d'une
          // commande AT, d'un SMS ou d'une reponse MQTT en attente.
          const bool modemWorkPending =
              modemTcpBusy || modemExpectSendPrompt || modemWaitingResponse ||
              smsTxBusy || smsQueueCount > 0 ||
              mqttAckPending || mqttResponsePending || mqttCommandPending ||
              recoveryFrame.length() > 0;

          if (modemWorkPending) {
            setModemPower(true);
          } else {
            normalBufferRecoveryActive = false;
            setModemPower(false);
            addLog("NORMAL : BUFFER VIDE -> MODEM ARRETE");
          }
        } else {
          setModemPower(true);
        }
      } else if (bufferPending >= 100) {
        normalBufferRecoveryActive = true;
        setModemPower(true);
        addLog("NORMAL : BUFFER 100 -> RECUPERATION");
      } else {
        setModemPower(false);
      }
      return;
    }

    normalBufferRecoveryActive = false;

    if (stopDelayDone) {
      if (bufferPending == 0) {
        normalPostStopRecoveryActive = false;
        normalPostStopRecoveryStart = 0;
        setGPSPower(false);
        setModemPower(false);
        enterConfiguredDeepSleep("NORMAL / STOP / NO BUFFER");
        return;
      }

      if (!normalPostStopRecoveryActive) {
        normalPostStopRecoveryActive = true;
        normalPostStopRecoveryStart = now;
        addLog("NORMAL : RECUPERATION APRES ARRET / FENETRE 2 MIN");
      }

      if (bufferPending == 0) {
        normalPostStopRecoveryActive = false;
        normalPostStopRecoveryStart = 0;
        setGPSPower(false);
        setModemPower(false);
        enterConfiguredDeepSleep("NORMAL / BUFFER VIDE");
        return;
      }

      if (now - normalPostStopRecoveryStart >= NORMAL_POST_STOP_RECOVERY_MAX_MS) {
        normalPostStopRecoveryActive = false;
        normalPostStopRecoveryStart = 0;
        setModemPower(false);
        setGPSPower(false);
        enterConfiguredDeepSleep("NORMAL / RECUPERATION 2 MIN");
        return;
      }

      setModemPower(true);
      return;
    }

    normalPostStopRecoveryActive = false;
    normalPostStopRecoveryStart = 0;
    setModemPower(false);
    return;
  }

  // ---------------------------------------------------------------
  // ECO
  // ---------------------------------------------------------------
  // Le modem ne fonctionne qu'après arrêt + délai.
  // Si un mouvement reprend pendant la récupération, taskMovement()
  // coupe le modem immédiatement et l'enregistrement continue.
  if (cfg.energyMode == 1) {
    if (moving) {
      setModemPower(false);
      return;
    }

    if (stopDelayDone && bufferPending > 0) {
      setModemPower(true);
      return;
    }

    if (stopDelayDone && bufferPending == 0) {
      setGPSPower(false);
      setModemPower(false);
      enterConfiguredDeepSleep("ECO / STOP / NO BUFFER");
      return;
    }

    setModemPower(false);
    return;
  }

  // ---------------------------------------------------------------
  // PERFORMANCE
  // ---------------------------------------------------------------
  // GPS toujours actif dans ce mode.
  // Pendant le mouvement : communication active + envoi temps reel.
  // Apres arret confirme : aucune nouvelle position n'est creee ; seules
  // les positions deja presentes dans le buffer sont transmises.
  // Lorsque le buffer est vide, seule la communication est coupee.
  if (cfg.energyMode == 2) {
    setGPSPower(true);

    if (moving && !stopConfirmed) {
      setModemPower(true);
      return;
    }

    if (stopConfirmed) {
      if (bufferPending > 0) {
        setModemPower(true);
        return;
      }

      if (modemPower && !modemTcpBusy && !modemExpectSendPrompt && !smsTxBusy) {
        setModemPower(false);
      }
      return;
    }

    // Etat de securite : GPS reste actif, mais la communication ne doit
    // pas rester allumee sans mouvement ni donnees a transmettre.
    setModemPower(false);
    return;
  }

  // ---------------------------------------------------------------
  // ULTRA ECO
  // ---------------------------------------------------------------
  // Hors CHECK, le modem reste OFF.
  if (stopDelayDone && bufferPending > 0) {
    setModemPower(true);
  } else if (modemPower && !modemTcpBusy && !modemExpectSendPrompt && !smsTxBusy) {
    setModemPower(false);
  }
}

void taskPower() {
  if (modemOTAActive) return;
  taskModeManager();
}

void taskWatchdog() {
  // Watchdog matériel explicite : alimentation logique continue sans boucle bloquante.
  static unsigned long lastFeed = 0;
  const unsigned long now = millis();
  if (now - lastFeed >= 1000UL) {
    lastFeed = now;
    ESP.wdtFeed();
  }
}

void taskLogger() {
  // Les événements restent dans le buffer circulaire Web pour ne pas écrire
  // continuellement en Flash.
}

// =========================
// SETUP
// =========================
void setup() {

  // UART matériel réservé à l'A7670E.
  // Si ton module utilise un autre débit, modifier MODEM_BAUD.
  // Réserve un RX UART plus large pour les payloads binaires HTTPREAD.
  // Important pour les blocs de 1024 octets reçus rapidement du A7670E.
  Serial.setRxBufferSize(OTA_MODEM_RX_BUFFER_SIZE);
  Serial.begin(MODEM_BAUD);
  ESP.wdtEnable(WDTO_8S);
  ESP.wdtFeed();
  gpsSerial.begin(9600);

  bootTime = millis();
  wakeStartedAt = bootTime;
  wakeVibrationWindow = true;
  motionConfirmationActive = false;
  activeTracking = false;
  vibrationTransitions = 0;
  vibrationLastState = HIGH;
  vibrationLastEdgeAt = bootTime;
  sleepRequested = false;

  // On force l'état OFF (HIGH) avant même de passer les broches en
  // OUTPUT, pour éviter toute impulsion d'allumage parasite au boot
  // (la logique est active-low : LOW = ON).
  digitalWrite(GPS_POWER, HIGH);
  digitalWrite(MODEM_POWER, HIGH);

  pinMode(GPS_POWER, OUTPUT);
  pinMode(MODEM_POWER, OUTPUT);

  pinMode(VIBRATION, INPUT_PULLUP);

  setGPSPower(false);
  setModemPower(false);

  // Le Serial matériel est réservé à l'A7670E.
  // Aucun texte de debug n'est envoyé ici pour éviter d'injecter
  // des caractères parasites dans l'UART du modem.
  if (!SPIFFS.begin()) {
    addLog("SPIFFS : ERROR");
  } else {
    // Nettoyage des temporaires restés après une ancienne tentative.
    // Les données utiles restent intactes.
    cleanupTemporaryFiles();
    addLog("SPIFFS : OK");
  }

  loadConfig();
  batteryAlertSent = loadBatteryAlertState();
  // V6 : ne jamais allumer le modem au démarrage uniquement pour lire l'IMEI.
  // L'IMEI est acquis de manière opportuniste lorsque le modem est réellement requis.
  apSSID = (cfg.imei.length() == 15) ? cfg.imei : String("NUMOTRONIC-SMART01");
  countBuffer();
  lastMovement = millis();
  lastPositionEvent = millis();
  longIntervalStart = millis();
  initializeSleepSchedule();
  loadPendingLocationRequest();

  // Voyant de signalisation sur la sortie 2.
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF_LEVEL);

  // Wi-Fi OFF par défaut au démarrage.
  // Activation uniquement par SMS ou appui long de 4 s sur GPIO0.
  pinMode(WIFI_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF_LEVEL);
  wifiEnabled = false;
  wifiStartedAt = 0;
  addLog("WiFi OFF au demarrage");

  setupRoutes();

  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "PAGE INTROUVABLE");
  });

  server.begin();

  //startWiFi();

  addLog("WEB SERVER READY");
  // Le Wi-Fi reste OFF au démarrage : activation par SMS WIFI ON ou appui long.
  // Les requêtes Web seront traitées uniquement lorsque le point d’accès est actif.

  // Réveil : vibration habituelle ou cycle périodique de communication selon le mode.
  wakeStartedAt = millis();
  motionConfirmationActive = false;
  vibrationTransitions = 0;
  vibrationLastState = (digitalRead(VIBRATION) == HIGH) ? HIGH : LOW;
  vibrationLastEdgeAt = millis();
  sleepRequested = false;
  communicationWakeStartedAt = 0;
  communicationWakeWindowMs = 0;

  // REVEIL PERIODIQUE DE COMMUNICATION COMMUN A NORMAL, ECO,
  // PERFORMANCE et ULTRA ECO. TEST et PERMANENCE restent continus.
  // Le compteur RTC progresse a chaque sortie de Deep-Sleep.
  scheduledGrandWake = (cfg.energyMode <= 3 &&
                        sleepWakeMinute > 0 &&
                        cfg.longIntervalMin > 0 &&
                        (sleepWakeMinute % cfg.longIntervalMin) == 0);
  scheduledSmsWake = false;
  scheduledCommunicationWake = scheduledGrandWake;

  if (scheduledCommunicationWake) {
    wakeVibrationWindow = false;
    // CHECK ne declenche pas automatiquement le suivi/memoire.
    // Le suivi ne commence que si un mouvement > 6 km/h est detecte.
    activeTracking = false;
    communicationWakeWindowMs = (unsigned long)max((uint16_t)1, cfg.heartbeatInterval) * 1000UL;
  } else {
    // ULTRA ECO : aucun reveil vibration. Seuls les CHECK sont autorises.
    wakeVibrationWindow = (cfg.energyMode != 3 && cfg.energyMode != 4);
    activeTracking = (cfg.energyMode == 4);
  }

  if (cfg.energyMode == 4) {
    setGPSPower(true);
    setModemPower(true);
    addLog("SYSTEM READY / MODE TEST CONTINU / GPS+MODEM ON");
  } else if (cfg.energyMode == 5) {
    setGPSPower(true);
    setModemPower(true);
    permanenceModemSleep = false;
    wakeVibrationWindow = false;
    activeTracking = true;
    addLog("SYSTEM READY / MODE PERMANENCE / GPS+MODEM ON / COMMANDES SMS+MQTT DISPONIBLES");
  } else if (scheduledCommunicationWake) {
    setGPSPower(true);
    setModemPower(false); // active par taskScheduledCommunication()
    addLog("REVEIL PERIODIQUE / GPS + COMMANDES + TCP / MAINTIEN=" + String(cfg.heartbeatInterval) + "s");
  } else {
    setGPSPower(false);
    setModemPower(false);
    addLog("SYSTEM READY / SURVEILLANCE VIBRATION");
  }

  // Serial est réservé au modem A7670E.
  // Les informations de démarrage sont disponibles via la page Web.
}

// =========================
// TASKS
// =========================
void taskOTAWeb() {
  // Une demande OTA issue de l'interface Web n'est exécutée que si le Wi-Fi
  // est actif. Les mises à jour 4G demandées par SMS gardent leur propre tâche.
  if (!wifiEnabled || !otaWebRequestPending || otaOTAInProgress) return;

  String url = otaWebRequestURL;
  otaWebRequestURL = "";
  otaWebRequestPending = false;
  otaWebStreamActive = false;

  otaWebLog("DÉBUT DE LA SESSION OTA");
  bool ok = updateFirmwareFrom4G(url);

  if (ok) {
    otaWebLog("SUCCÈS : firmware installé");
    otaRestartPending = true;
    otaRestartAt = millis() + 1000UL;
  } else {
    otaWebLog("ERREUR OTA : " + otaStatusMessage + " / " + otaLastError);
  }
}

void taskWeb() {
  // Le serveur HTTP ne doit pas travailler lorsque le Wi-Fi est désactivé.
  if (!wifiEnabled) return;
  server.handleClient();
}

void taskGPS() {

  if (modemOTAActive) return;
  if (!gpsPower)
    return;

  if (millis() - lastGPS >= 20) {
    lastGPS = millis();
    readGPS();
  }
}

// =========================
// WIFI BUTTON MANAGER
// GPIO0 -> GND, pression longue
// =========================
void startWiFi() {
  if (wifiEnabled) return;
  wifiEnabled = true;
  wifiStartedAt = millis();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, AP_PASS);
  addLog("WiFi ON");
}

void stopWiFi() {
  if (!wifiEnabled) return;
  wifiEnabled = false;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiStartedAt = 0;
  addLog("WiFi OFF");
}

void taskWiFiButton() {
  bool down = (digitalRead(WIFI_BUTTON_PIN) == LOW);

  // Wi-Fi AP de configuration : disponible au démarrage puis coupé
  // automatiquement après 5 min pour préserver la batterie.
  if (wifiEnabled && !down && wifiStartedAt && millis() - wifiStartedAt >= WIFI_AUTO_OFF_MS) {
    stopWiFi();
  }

  // Nouveau appui : démarrage du chronomètre.
  if (down && !wifiButtonWasDown) {
    wifiButtonStart = millis();
    wifiButtonActionDone = false;
  }

  // Après exactement 4 s maintenus : inversion ON/OFF une seule fois.
  if (down && wifiButtonWasDown && !wifiButtonActionDone) {
    if (millis() - wifiButtonStart >= WIFI_LONG_PRESS_MS) {
      wifiButtonActionDone = true;
      if (wifiEnabled) stopWiFi();
      else startWiFi();
    }
  }

  // Relâchement : réarmement pour le prochain appui.
  if (!down && wifiButtonWasDown) {
    wifiButtonActionDone = false;
  }

  wifiButtonWasDown = down;
}

// ============================================================
// SIM HOT-SWAP : reset materiel A7670E via GPIO13
// ============================================================
void taskSimHotSwapReset() {
  const unsigned long now = millis();


  if (scheduledCommunicationWake &&
      (cfg.energyMode == 0 ||
       cfg.energyMode == 1 ||
       cfg.energyMode == 3)) {
    return;
  }

  // Phase de coupure puis remise sous tension : taskModem() est suspendue
  // tant que simResetInProgress reste vrai.
  if (simResetInProgress) {
    if (simRecoveryPhase == 1 &&
        now - simResetStartedAt >= SIM_POWER_OFF_MS) {
      modemStage = MODEM_STAGE_BOOT;
      modemStageSince = now;
      setModemPower(true);  // LOW = alimentation ON
      simRecoveryPhase = 2;
      simResetStartedAt = now;
      lastSimPoll = 0;
      addLog("SIM HOT-SWAP : ALIMENTATION MODEM RETABLIE");
    } else if (simRecoveryPhase == 2 &&
               now - simResetStartedAt >= SIM_POWER_BOOT_WAIT_MS) {
      simResetInProgress = false;
      simRecoveryPhase = 0;
      simAbsentSince = 0;
      modemStage = MODEM_STAGE_BOOT;
      modemStageSince = now;
      lastSimPoll = 0;
      addLog("SIM HOT-SWAP : REPRISE INITIALISATION MODEM");
    }
    return;
  }

  if (simPresent && simReady) {
    simAbsentSince = 0;
    return;
  }

  if (simState != "SIM ABSENTE") {
    simAbsentSince = 0;
    return;
  }

  if (simAbsentSince == 0) simAbsentSince = now;
  if (now - simAbsentSince < SIM_ABSENCE_CONFIRM_MS) return;
  if (simResetLastAttemptAt != 0 &&
      now - simResetLastAttemptAt < SIM_RESET_RETRY_MS) return;

  networkRegistered = false;
  packetServiceAttached = false;
  modemDataReady = false;
  tcpServiceOpen = false;
  tcpNetOpening = false;
  modemWaitingResponse = false;
  link1.connected = link1.opening = link1.sending = link1.loginSent = false;
  link2.connected = link2.opening = link2.sending = link2.loginSent = false;

  simResetInProgress = true;
  simRecoveryPhase = 1;
  simResetStartedAt = now;
  simResetLastAttemptAt = now;
  modemState = "COUPURE MODEM - SIM ABSENTE";
  setModemPower(false); // HIGH = alimentation OFF
  addLog("SIM ABSENTE PERSISTANTE : COUPURE MODEM_POWER GPIO12 3s");
}

void loop() {
  ESP.wdtFeed();
  if (otaRestartPending && (int32_t)(millis() - otaRestartAt) >= 0) {
    otaRestartPending = false;
    delay(100);
    ESP.restart();
  }

  // Priorite : Web + UART modem, puis logique non bloquante.
  
  readBattery();
  taskWeb();
  taskOTAWeb();
  taskModem();
  taskSimHotSwapReset();
  taskPendingSMSOTA();
  taskGPS();
  taskNetwork();
  taskMQTT();
  taskServer();
  taskSendRecovery();
  taskScheduledCommunication();
  taskPendingLocationSMS();
  taskMovement();
  taskPositionManager();
  taskBufferMaintenance();
  taskPower();
  taskTestModemRestart();
  taskWiFiButton();
  taskStatusLED();
  taskLogger();
  taskWatchdog();
  yield();
}

// ============================================================
// DEEP-SLEEP IMPLEMENTATION
// ============================================================
// ============================================================
// MÉMOIRE RTC + VEILLE PROFONDE PÉRIODIQUE
// ============================================================
uint32_t sleepRTCChecksum(const SleepRTCData &d) {
  uint32_t c = 2166136261UL;
  const uint8_t *p = reinterpret_cast<const uint8_t*>(&d);
  for (size_t i = 0; i < sizeof(SleepRTCData) - sizeof(uint32_t); ++i) {
    c ^= p[i];
    c *= 16777619UL;
  }
  return c;
}

void saveSleepRTC(uint32_t nextMinute) {
  SleepRTCData d;
  d.magic = SLEEP_RTC_MAGIC;
  d.nextWakeMinute = nextMinute;
  d.checksum = 0;
  d.checksum = sleepRTCChecksum(d);
  ESP.rtcUserMemoryWrite(0, reinterpret_cast<uint32_t*>(&d), sizeof(d));
}

void initializeSleepSchedule() {
  sleepWakeMinute = 0;
  sleepRtcValid = false;

  String rr = ESP.getResetReason();
  bool deepWake = (rr.indexOf("Deep-Sleep") >= 0 || rr.indexOf("Deep Sleep") >= 0);

  SleepRTCData d;
  memset(&d, 0, sizeof(d));
  if (deepWake && ESP.rtcUserMemoryRead(0, reinterpret_cast<uint32_t*>(&d), sizeof(d))) {
    uint32_t expected = sleepRTCChecksum(d);
    if (d.magic == SLEEP_RTC_MAGIC && d.checksum == expected) {
      sleepWakeMinute = d.nextWakeMinute;
      sleepRtcValid = true;
    }
  }

  // Toute mise sous tension ou réinitialisation volontaire recommence le compteur.
  if (!sleepRtcValid) sleepWakeMinute = 0;
}

void enterConfiguredDeepSleep(const char* reason) {
  // MODE TEST / PERMANENCE : aucune veille profonde.
  if (cfg.energyMode == 4 || cfg.energyMode == 5) {
    setGPSPower(true);
    setModemPower(true);
    sleepRequested = false;
    return;
  }

  // NORMAL/ECO/PERFORMANCE : reveil de base toutes les 60 s.
  // ULTRA ECO : aucun reveil intermediaire, reveil directement au prochain CHECK.
  uint32_t sleepMinutes = (cfg.energyMode == 3 && cfg.longIntervalMin > 0)
                          ? cfg.longIntervalMin : 1UL;
  uint32_t nextMinute = sleepWakeMinute + sleepMinutes;
  saveSleepRTC(nextMinute);

  addLog(String("VEILLE PROFONDE 1 MIN - ") +
         (reason ? String(reason) : String("INCONNU")) +
         " / PROCHAIN CYCLE=" + String(nextMinute));

  setModemPower(false);
  setGPSPower(false);

  modemTcpBusy = false;
  modemExpectSendPrompt = false;
  modemWaitingResponse = false;
  tcpServiceOpen = false;
  tcpNetOpening = false;
  activeTracking = false;
  moving = false;
  stopConfirmed = false;
  motionConfirmationActive = false;
  wakeVibrationWindow = true;
  vibrationTransitions = 0;
  scheduledCommunicationWake = false;
  scheduledGrandWake = false;
  scheduledSmsWake = false;
  communicationWakeStartedAt = 0;
  communicationWakeWindowMs = 0;

  digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF_LEVEL);
  delay(100);
  Serial.flush();

  // GPIO16 (D0) doit être relié à RST pour le réveil périodique.
  ESP.deepSleep((uint64_t)sleepMinutes * 60ULL * 1000000ULL);
  delay(100);
}
