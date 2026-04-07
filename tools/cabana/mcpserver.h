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

  // Utility functions
  QJsonArray getAvailableMessages();
  QJsonArray getSignalsForMessage(const QString &message_name);
  QString formatCanMessage(const CanEvent &event);
  QJsonObject createError(const QString &code, const QString &message);

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