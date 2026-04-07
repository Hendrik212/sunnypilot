#pragma once

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <memory>

#include "tools/cabana/streams/abstractstream.h"
#include "tools/cabana/dbc/dbcmanager.h"

class MainWindow;  // Forward declaration

class McpServer : public QObject {
  Q_OBJECT

public:
  explicit McpServer(QObject *parent = nullptr);
  ~McpServer();

  // Server control
  bool startServer(quint16 port = 3001);
  void stopServer();
  bool isRunning() const;

  // Cabana integration
  void setStream(AbstractStream *new_stream);
  void setDbcManager(DBCManager *new_dbc_manager);
  void setMainWindow(MainWindow *main_window);

private slots:
  void onNewConnection();
  void onClientDisconnected();
  void onDataReceived();

private:
  // MCP Protocol handling
  void handleMessage(const QJsonObject &message, QTcpSocket *client);
  void sendResponse(const QJsonObject &response, QTcpSocket *client);
  void sendNotification(const QJsonObject &notification, QTcpSocket *client);

  // MCP Methods
  QJsonObject handleInitialize(const QJsonObject &params);
  QJsonObject handleToolsList();
  QJsonObject handleToolsCall(const QJsonObject &params);
  QJsonObject handleResourcesList();
  QJsonObject handleResourcesRead(const QJsonObject &params);

  // Tool implementations
  QJsonObject executeAnalyzeCanMessages(const QJsonObject &args);
  QJsonObject executeGetDbcInfo(const QJsonObject &args);
  QJsonObject executeDecodeMessage(const QJsonObject &args);
  QJsonObject executeGetSignalValues(const QJsonObject &args);
  QJsonObject executeSearchSignals(const QJsonObject &args);
  QJsonObject executeExportData(const QJsonObject &args);
  QJsonObject executeGetRouteInfo(const QJsonObject &args);
  QJsonObject executeSelectMessage(const QJsonObject &args);
  QJsonObject executeCreateSignal(const QJsonObject &args);
  QJsonObject executeAnalyzeSignalPatterns(const QJsonObject &args);

  // Signal resolution: find a signal by name across all DBC files.
  // Returns the Signal pointer and populates msg_id with the parent message.
  struct ResolvedSignal {
    const cabana::Signal *sig = nullptr;
    MessageId msg_id = {};
  };
  ResolvedSignal resolveSignal(const QString &name) const;

  // Parse hex message ID string → uint32_t. Throws on failure.
  uint32_t parseHexId(const QString &str) const;

  // Parse time_range string → (start, end) seconds. Supports "all", "120.5-135.2".
  std::pair<double, double> parseTimeRange(const QString &str) const;

  // Utility functions
  QJsonArray getAvailableMessages();
  QJsonArray getSignalsForMessage(const QString &message_name);
  QString formatCanMessage(const CanEvent &event);
  QJsonObject createError(const QString &code, const QString &message);

  // Wrap result in MCP content response
  QJsonObject makeTextResponse(const QJsonValue &result);

  QTcpServer *server;
  QList<QTcpSocket*> clients;
  AbstractStream *stream;
  DBCManager *dbc_manager;
  MainWindow *main_window;

  // MCP Server capabilities
  static const QString SERVER_NAME;
  static const QString SERVER_VERSION;
  static const QJsonArray SUPPORTED_TOOLS;
};
