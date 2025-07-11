/*
 * communication_manager.h
 * Communication Manager Class for nanoELS H5
 * 
 * Handles WiFi connectivity, WebSocket communication, PS2 keyboard input,
 * and web interface management with real-time data exchange.
 */

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <PS2KeyAdvanced.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config/hardware_config.h"
#include "core/axis_controller.h"
#include "core/spindle_encoder.h"

// Communication configuration
struct CommunicationConfig {
    // WiFi settings
    const char* ssid;                // WiFi SSID
    const char* password;            // WiFi password
    const char* hostname;            // Device hostname
    int wifiTimeout;                 // WiFi connection timeout (ms)
    
    // Web server settings
    int webServerPort;               // Web server port
    int webSocketPort;               // WebSocket port
    int maxConnections;              // Maximum simultaneous connections
    int bufferSize;                  // Communication buffer size
    
    // PS2 keyboard settings
    int keyboardDataPin;             // PS2 data pin
    int keyboardClockPin;            // PS2 clock pin
    bool keyboardEnabled;            // Enable PS2 keyboard
    int keyboardTimeout;             // Keyboard timeout (ms)
    
    // Communication settings
    bool enableWebInterface;          // Enable web interface
    bool enableWebSocket;            // Enable WebSocket communication
    bool enableSerialDebug;          // Enable serial debug output
    int updateInterval;              // Communication update interval (ms)
};

// Communication state structure
struct CommunicationState {
    bool wifiConnected;               // WiFi connection status
    bool webServerRunning;           // Web server status
    bool webSocketRunning;           // WebSocket status
    bool keyboardConnected;          // PS2 keyboard status
    unsigned long lastUpdateTime;    // Last update time
    int activeConnections;           // Active web connections
    int messageCount;                // Total messages processed
    char lastError[64];              // Last error message
};

// Keyboard event structure
struct KeyboardEvent {
    uint16_t keyCode;                // PS2 key code
    bool keyDown;                    // Key press state
    bool keyRepeat;                  // Key repeat state
    unsigned long timestamp;         // Event timestamp
    char keyChar;                    // ASCII character (if applicable)
};

// WebSocket message structure
struct WebSocketMessage {
    uint8_t num;                     // Client number
    WStype_t type;                   // Message type
    uint8_t* payload;                // Message payload
    size_t length;                   // Payload length
    unsigned long timestamp;         // Message timestamp
};

class CommunicationManager {
private:
    // Configuration
    CommunicationConfig config;
    CommunicationState state;
    
    // WiFi and network
    WiFiServer* webServer;
    WebSocketsServer* webSocketServer;
    
    // PS2 keyboard
    PS2KeyAdvanced* keyboard;
    
    // Thread safety
    SemaphoreHandle_t commMutex;
    QueueHandle_t keyboardQueue;
    QueueHandle_t webSocketQueue;
    
    // Communication buffers
    char* incomingBuffer;
    char* outgoingBuffer;
    size_t bufferSize;
    
    // Message processing
    volatile bool messagePending;
    volatile unsigned long lastMessageTime;
    volatile int messageCounter;
    
    // Private methods
    void initializeWiFi();
    void initializeWebServer();
    void initializeWebSocket();
    void initializeKeyboard();
    void processWiFiEvents();
    void processWebSocketEvents();
    void processKeyboardEvents();
    void handleWebSocketMessage(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void handleWebServerRequest();
    void handleKeyboardEvent(const KeyboardEvent& event);
    
    // Web server handlers
    void handleRoot();
    void handleStatus();
    void handleGCodeList();
    void handleGCodeGet();
    void handleGCodeAdd();
    void handleGCodeRemove();
    void handleNotFound();
    
    // WebSocket handlers
    void handleWebSocketConnect(uint8_t num);
    void handleWebSocketDisconnect(uint8_t num);
    void handleWebSocketError(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    
    // Message processing
    void processIncomingMessage(const char* message);
    void sendOutgoingMessage(const char* message);
    void broadcastStatus();
    void sendAxisStatus();
    void sendSpindleStatus();
    void sendSystemStatus();
    
public:
    CommunicationManager();
    CommunicationManager(const CommunicationConfig& configuration);
    ~CommunicationManager();
    
    // Initialization and configuration
    bool initializeCommunication();
    bool configureCommunication(const CommunicationConfig& newConfig);
    CommunicationConfig getConfiguration() const;
    bool validateConfiguration();
    
    // WiFi management
    bool connectWiFi();
    void disconnectWiFi();
    bool isWiFiConnected() const;
    const char* getWiFiSSID() const;
    IPAddress getWiFiIP() const;
    int getWiFiRSSI() const;
    
    // Web server management
    bool startWebServer();
    void stopWebServer();
    bool isWebServerRunning() const;
    int getActiveConnections() const;
    void broadcastToClients(const char* message);
    
    // WebSocket management
    bool startWebSocket();
    void stopWebSocket();
    bool isWebSocketRunning() const;
    void sendToClient(uint8_t clientNum, const char* message);
    void broadcastToWebSocket(const char* message);
    
    // PS2 keyboard management
    bool initializeKeyboard();
    void processKeyboardInput();
    bool isKeyboardConnected() const;
    bool hasKeyboardEvent() const;
    KeyboardEvent getNextKeyboardEvent();
    
    // Communication processing
    void processCommunication();
    void handleWebRequests();
    void handleWebSocketMessages();
    void processKeyboardInput();
    
    // Message handling
    void sendStatusUpdate();
    void sendAxisUpdate();
    void sendSpindleUpdate();
    void sendSystemUpdate();
    void sendError(const char* error);
    
    // Communication state queries
    bool isConnected() const;
    bool isServerRunning() const;
    bool isKeyboardConnected() const;
    int getMessageCount() const;
    const char* getLastError() const;
    unsigned long getUptimeSeconds() const;
    
    // Thread-safe operations
    bool lockCommunication(TickType_t timeout = portMAX_DELAY);
    void unlockCommunication();
    
    // Utility functions
    void formatStatusMessage(char* buffer, size_t bufferSize) const;
    void formatAxisMessage(char* buffer, size_t bufferSize) const;
    void formatSpindleMessage(char* buffer, size_t bufferSize) const;
    void formatSystemMessage(char* buffer, size_t bufferSize) const;
    
    // Enhanced communication features
    void enableWebInterface(bool enable);
    void enableWebSocket(bool enable);
    void enableKeyboard(bool enable);
    void setUpdateInterval(int interval);
    
    // File system integration
    bool serveFile(const char* filename, const char* contentType);
    bool listGCodeFiles(char* buffer, size_t bufferSize);
    bool getGCodeFile(const char* filename, char* buffer, size_t bufferSize);
    bool saveGCodeFile(const char* filename, const char* content);
    bool deleteGCodeFile(const char* filename);
    
    // Diagnostics and monitoring
    bool performSelfTest();
    bool testWiFiConnection();
    bool testWebServer();
    bool testWebSocket();
    bool testKeyboard();
    float getMessageRate() const;
    
    // Command processing
    void processCommand(const char* command);
    void handleKeyCode(uint16_t keyCode);
    void handleSystemCommand(const char* command);
    void handleAxisCommand(const char* command);
    void handleSpindleCommand(const char* command);
    
private:
    // Communication buffers
    char* tempBuffer;
    size_t tempBufferSize;
    
    // Event queues
    static const int MAX_KEYBOARD_EVENTS = 20;
    static const int MAX_WEBSOCKET_MESSAGES = 50;
    KeyboardEvent keyboardEvents[MAX_KEYBOARD_EVENTS];
    WebSocketMessage webSocketMessages[MAX_WEBSOCKET_MESSAGES];
    int keyboardEventCount;
    int webSocketMessageCount;
    
    // Status tracking
    unsigned long lastStatusUpdate;
    unsigned long lastAxisUpdate;
    unsigned long lastSpindleUpdate;
    bool statusChanged;
    bool axisChanged;
    bool spindleChanged;
    
    // Error handling
    void handleCommunicationError(const char* error);
    void resetErrorCount();
    void incrementErrorCount();
};

// Global communication manager instance
extern CommunicationManager* g_communicationManager;

// Communication utility functions
inline bool isValidPort(int port) {
    return port > 0 && port <= 65535;
}

inline bool isValidBufferSize(size_t size) {
    return size > 0 && size <= 65536;
}

inline bool isValidSSID(const char* ssid) {
    return ssid && strlen(ssid) > 0 && strlen(ssid) <= 32;
}

// Communication command macros
#define COMM_SEND_STATUS() do { \
    if (g_communicationManager) { \
        g_communicationManager->sendStatusUpdate(); \
    } \
} while(0)

#define COMM_BROADCAST(msg) do { \
    if (g_communicationManager) { \
        g_communicationManager->broadcastToClients(msg); \
    } \
} while(0)

#define COMM_SEND_ERROR(err) do { \
    if (g_communicationManager) { \
        g_communicationManager->sendError(err); \
    } \
} while(0)

// WebSocket event handlers
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// Web server request handlers
void handleWebRoot();
void handleWebStatus();
void handleWebGCode();
void handleWebNotFound();