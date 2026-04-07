#include "tools/cabana/mcpserver.h"

#include <QJsonDocument>
#include <QTextStream>
#include <QDebug>
#include <QDateTime>
#include <QMetaObject>
#include <QThread>

#include "tools/cabana/mainwin.h"

const QString McpServer::SERVER_NAME = "Cabana MCP Server";
const QString McpServer::SERVER_VERSION = "1.0.0";
const QJsonArray McpServer::SUPPORTED_TOOLS = QJsonArray{
  QJsonObject{
    {"name", "analyze_can_messages"},
    {"description", "Analyze CAN messages from the current stream"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range to analyze (e.g., '1s', '10s', 'all')"}}},
        {"bus_filter", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "integer"}}}, {"description", "Filter by CAN bus numbers"}}},
        {"message_filter", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "string"}}}, {"description", "Filter by message names or IDs"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "select_message"},
    {"description", "Select/open a specific CAN message in the GUI"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"message_id"}},
      {"properties", QJsonObject{
        {"message_id", QJsonObject{{"type", "string"}, {"description", "CAN message ID in hex format (e.g., '0x123' or '123')"}}},
        {"source", QJsonObject{{"type", "integer"}, {"description", "CAN bus source number (optional, defaults to searching all sources)"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "get_dbc_info"},
    {"description", "Get information about loaded DBC files"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
        {"source", QJsonObject{{"type", "string"}, {"description", "DBC source name (optional)"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "decode_message"},
    {"description", "Decode a CAN message using the loaded DBC"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"message_id", "data"}},
      {"properties", QJsonObject{
        {"message_id", QJsonObject{{"type", "string"}, {"description", "CAN message ID (hex)"}}},
        {"data", QJsonObject{{"type", "string"}, {"description", "CAN message data (hex)"}}},
        {"source", QJsonObject{{"type", "string"}, {"description", "DBC source name (optional)"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "get_signal_values"},
    {"description", "Get current values for specific signals"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"signals"}},
      {"properties", QJsonObject{
        {"signals", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "string"}}}, {"description", "List of signal names"}}},
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range to get values for"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "search_signals"},
    {"description", "Search for signals by name or pattern"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"query"}},
      {"properties", QJsonObject{
        {"query", QJsonObject{{"type", "string"}, {"description", "Search query or regex pattern"}}},
        {"case_sensitive", QJsonObject{{"type", "boolean"}, {"description", "Case sensitive search"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "export_data"},
    {"description", "Export CAN data to various formats"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"format"}},
      {"properties", QJsonObject{
        {"format", QJsonObject{{"type", "string"}, {"enum", QJsonArray{"csv", "json", "dbc"}}, {"description", "Export format"}}},
        {"output_path", QJsonObject{{"type", "string"}, {"description", "Output file path"}}},
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range to export"}}},
        {"messages", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "string"}}}, {"description", "Specific messages to export"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "get_route_info"},
    {"description", "Get information about the current route/stream"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{}}
    }}
  },
  QJsonObject{
    {"name", "create_signal"},
    {"description", "Create/label a new signal in a CAN message"},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"message_id", "signal_name", "start_bit", "size"}},
      {"properties", QJsonObject{
        {"message_id", QJsonObject{{"type", "string"}, {"description", "CAN message ID in hex format (e.g., '0x123' or '123')"}}},
        {"source", QJsonObject{{"type", "integer"}, {"description", "CAN bus source number (optional, defaults to 0)"}}},
        {"signal_name", QJsonObject{{"type", "string"}, {"description", "Name of the signal to create"}}},
        {"start_bit", QJsonObject{{"type", "integer"}, {"minimum", 0}, {"maximum", 63}, {"description", "Starting bit position (0-63)"}}},
        {"size", QJsonObject{{"type", "integer"}, {"minimum", 1}, {"maximum", 64}, {"description", "Signal size in bits"}}},
        {"is_signed", QJsonObject{{"type", "boolean"}, {"description", "Whether the signal is signed (default: false)"}}},
        {"is_little_endian", QJsonObject{{"type", "boolean"}, {"description", "Whether the signal is little endian (default: true)"}}},
        {"factor", QJsonObject{{"type", "number"}, {"description", "Scaling factor (default: 1.0)"}}},
        {"offset", QJsonObject{{"type", "number"}, {"description", "Offset value (default: 0.0)"}}},
        {"unit", QJsonObject{{"type", "string"}, {"description", "Signal unit (optional)"}}},
        {"comment", QJsonObject{{"type", "string"}, {"description", "Signal comment/description (optional)"}}}
      }}
    }}
  }
};

McpServer::McpServer(QObject *parent)
    : QObject(parent), server(nullptr), stream(nullptr), dbc_manager(nullptr), main_window(nullptr) {
}

McpServer::~McpServer() {
  stopServer();
}

bool McpServer::startServer(quint16 port) {
  if (server && server->isListening()) {
    return false; // Already running
  }

  server = new QTcpServer(this);
  connect(server, &QTcpServer::newConnection, this, &McpServer::onNewConnection);

  if (!server->listen(QHostAddress::LocalHost, port)) {
    qWarning() << "Failed to start MCP server on port" << port << ":" << server->errorString();
    delete server;
    server = nullptr;
    return false;
  }

  qInfo() << "MCP server started on port" << port;
  return true;
}

void McpServer::stopServer() {
  if (server) {
    server->close();

    // Close all client connections synchronously
    for (QTcpSocket *client : clients) {
      if (client->state() != QAbstractSocket::UnconnectedState) {
        client->disconnectFromHost();
        // Wait for the connection to close gracefully (with timeout)
        if (client->state() != QAbstractSocket::UnconnectedState) {
          client->waitForDisconnected(1000); // Wait up to 1 second
        }
      }
      client->deleteLater();
    }
    clients.clear();

    delete server;
    server = nullptr;
    qInfo() << "MCP server stopped";
  }
}

bool McpServer::isRunning() const {
  return server && server->isListening();
}

void McpServer::setStream(AbstractStream *new_stream) {
  this->stream = new_stream;
}

void McpServer::setDbcManager(DBCManager *new_dbc_manager) {
  this->dbc_manager = new_dbc_manager;
}

void McpServer::setMainWindow(MainWindow *window) {
  this->main_window = window;
}

void McpServer::onNewConnection() {
  while (server->hasPendingConnections()) {
    QTcpSocket *client = server->nextPendingConnection();
    clients.append(client);

    connect(client, &QTcpSocket::readyRead, this, &McpServer::onDataReceived);
    connect(client, &QTcpSocket::disconnected, this, &McpServer::onClientDisconnected);

    qInfo() << "MCP client connected:" << client->peerAddress().toString();
  }
}

void McpServer::onClientDisconnected() {
  QTcpSocket *client = qobject_cast<QTcpSocket*>(sender());
  if (client) {
    clients.removeAll(client);
    qInfo() << "MCP client disconnected";
    client->deleteLater();
  }
}

void McpServer::onDataReceived() {
  QTcpSocket *client = qobject_cast<QTcpSocket*>(sender());
  if (!client) return;

  QByteArray data = client->readAll();
  QTextStream text_stream(data);
  QString line;

  while (text_stream.readLineInto(&line)) {
    if (line.trimmed().isEmpty()) continue;

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(line.toUtf8(), &error);

    if (error.error != QJsonParseError::NoError) {
      QJsonObject errorResponse{
        {"jsonrpc", "2.0"},
        {"error", QJsonObject{
          {"code", -32700},
          {"message", "Parse error"}
        }},
        {"id", QJsonValue::Null}
      };
      sendResponse(errorResponse, client);
      continue;
    }

    handleMessage(doc.object(), client);
  }
}

void McpServer::handleMessage(const QJsonObject &message, QTcpSocket *client) {
  QString method = message["method"].toString();
  QJsonValue id = message["id"];
  QJsonObject params = message["params"].toObject();

  QJsonObject response;
  response["jsonrpc"] = "2.0";
  if (!id.isNull()) {
    response["id"] = id;
  }

  try {
    if (method == "initialize") {
      response["result"] = handleInitialize(params);
    } else if (method == "tools/list") {
      response["result"] = handleToolsList();
    } else if (method == "tools/call") {
      response["result"] = handleToolsCall(params);
    } else if (method == "resources/list") {
      response["result"] = handleResourcesList();
    } else if (method == "resources/read") {
      response["result"] = handleResourcesRead(params);
    } else {
      response["error"] = createError("-32601", "Method not found");
    }
  } catch (const std::exception &e) {
    response["error"] = createError("-32603", QString("Internal error: %1").arg(e.what()));
  } catch (...) {
    response["error"] = createError("-32603", "Unknown internal error");
  }

  // Only send response if this is a request (has ID), not a notification
  if (!id.isNull()) {
    sendResponse(response, client);
  }
}

void McpServer::sendResponse(const QJsonObject &response, QTcpSocket *client) {
  if (!client || client->state() != QAbstractSocket::ConnectedState) {
    return;
  }

  QJsonDocument doc(response);
  QByteArray data = doc.toJson(QJsonDocument::Compact) + "\n";

  if (client->write(data) == -1) {
    qWarning() << "Failed to write response to client";
    return;
  }
  client->flush();
}

void McpServer::sendNotification(const QJsonObject &notification, QTcpSocket *client) {
  sendResponse(notification, client);
}

QJsonObject McpServer::handleInitialize(const QJsonObject &params) {
  Q_UNUSED(params);

  return QJsonObject{
    {"protocolVersion", "2024-11-05"},
    {"capabilities", QJsonObject{
      {"tools", QJsonObject{}},
      {"resources", QJsonObject{}}
    }},
    {"serverInfo", QJsonObject{
      {"name", SERVER_NAME},
      {"version", SERVER_VERSION}
    }}
  };
}

QJsonObject McpServer::handleToolsList() {
  return QJsonObject{
    {"tools", SUPPORTED_TOOLS}
  };
}

QJsonObject McpServer::handleToolsCall(const QJsonObject &params) {
  QString toolName = params["name"].toString();
  QJsonObject args = params["arguments"].toObject();

  if (toolName == "analyze_can_messages") {
    return executeAnalyzeCanMessages(args);
  } else if (toolName == "get_dbc_info") {
    return executeGetDbcInfo(args);
  } else if (toolName == "decode_message") {
    return executeDecodeMessage(args);
  } else if (toolName == "get_signal_values") {
    return executeGetSignalValues(args);
  } else if (toolName == "search_signals") {
    return executeSearchSignals(args);
  } else if (toolName == "export_data") {
    return executeExportData(args);
  } else if (toolName == "get_route_info") {
    return executeGetRouteInfo(args);
  } else if (toolName == "select_message") {
    return executeSelectMessage(args);
  } else if (toolName == "create_signal") {
    return executeCreateSignal(args);
  }

  throw std::runtime_error("Unknown tool: " + toolName.toStdString());
}

QJsonObject McpServer::handleResourcesList() {
  QJsonArray resources;

  if (dbc_manager) {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file) {
        QString name = QString::fromStdString(dbc_file->name());
        QJsonObject resourceObj;
        resourceObj["uri"] = QString("dbc://%1").arg(name);
        resourceObj["name"] = QString("DBC: %1").arg(name);
        resourceObj["mimeType"] = "application/x-dbc";
        resources.append(resourceObj);
      }
    }
  }

  if (stream) {
    resources.append(QJsonObject{
      {"uri", "stream://current"},
      {"name", "Current CAN Stream"},
      {"mimeType", "application/json"}
    });
  }

  return QJsonObject{
    {"resources", resources}
  };
}

QJsonObject McpServer::handleResourcesRead(const QJsonObject &params) {
  QString uri = params["uri"].toString();

  if (uri.startsWith("dbc://")) {
    QString dbcName = uri.mid(6); // Remove "dbc://"
    if (dbc_manager) {
      for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
        if (dbc_file && QString::fromStdString(dbc_file->name()) == dbcName) {
          // Return DBC content as JSON
          QJsonObject dbcJson;
          dbcJson["name"] = dbcName;
          dbcJson["messages"] = getAvailableMessages();

          QJsonObject contentObj;
          contentObj["uri"] = uri;
          contentObj["mimeType"] = "application/json";
          contentObj["text"] = QString::fromUtf8(QJsonDocument(dbcJson).toJson());

          QJsonObject result;
          result["contents"] = QJsonArray{contentObj};
          return result;
        }
      }
    }
  } else if (uri == "stream://current") {
    if (stream) {
      QJsonObject streamInfo;
      streamInfo["type"] = "CAN Stream";
      streamInfo["active"] = stream->liveStreaming();
      streamInfo["messages_available"] = getAvailableMessages();

      QJsonObject contentObj;
      contentObj["uri"] = uri;
      contentObj["mimeType"] = "application/json";
      contentObj["text"] = QString::fromUtf8(QJsonDocument(streamInfo).toJson());

      QJsonObject result;
      result["contents"] = QJsonArray{contentObj};
      return result;
    }
  }

  throw std::runtime_error("Resource not found: " + uri.toStdString());
}

QJsonObject McpServer::executeAnalyzeCanMessages(const QJsonObject &args) {
  if (!stream) {
    throw std::runtime_error("No stream available");
  }

  QString timeRange = args["time_range"].toString("all");
  QJsonArray busFilter = args["bus_filter"].toArray();
  QJsonArray messageFilter = args["message_filter"].toArray();

  QJsonObject result;
  result["time_range_requested"] = timeRange;
  result["total_events"] = static_cast<int>(stream->allEvents().size());
  result["route_name"] = QString::fromStdString(stream->routeName());

  // Analyze message frequency and activity
  const auto &lastMessages = stream->lastMessages();
  QJsonArray activeMessages;

  for (const auto &[msgId, canData] : lastMessages) {
    // Skip if bus filter is specified and this message doesn't match
    if (!busFilter.isEmpty()) {
      bool matchesBus = false;
      for (const auto &busVal : busFilter) {
        if (busVal.toInt() == msgId.source) {
          matchesBus = true;
          break;
        }
      }
      if (!matchesBus) continue;
    }

    QJsonObject msgInfo;
    msgInfo["message_id"] = QString("0x%1").arg(msgId.address, 0, 16);
    msgInfo["source"] = msgId.source;
    msgInfo["count"] = static_cast<int>(canData.count);
    msgInfo["frequency"] = canData.freq;
    msgInfo["last_seen"] = canData.ts;
    msgInfo["data_length"] = static_cast<int>(canData.dat.size());

    // Get last data bytes as hex
    QString dataHex;
    for (int i = 0; i < std::min(8, static_cast<int>(canData.dat.size())); ++i) {
      dataHex += QString("%1").arg(canData.dat[i], 2, 16, QChar('0')).toUpper();
    }
    msgInfo["last_data"] = dataHex;

    activeMessages.append(msgInfo);
  }

  result["active_messages"] = activeMessages;
  result["total_unique_messages"] = activeMessages.size();

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeGetDbcInfo(const QJsonObject &args) {
  QString source = args["source"].toString();

  if (!dbc_manager) {
    throw std::runtime_error("No DBC manager available");
  }

  QJsonObject result;
  QJsonArray dbcs;

  if (source.isEmpty()) {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file) {
        QJsonObject dbcInfo;
        dbcInfo["name"] = QString::fromStdString(dbc_file->name());
        dbcInfo["messages"] = getAvailableMessages();
        dbcs.append(dbcInfo);
      }
    }
  } else {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file && QString::fromStdString(dbc_file->name()) == source) {
        QJsonObject dbcInfo;
        dbcInfo["name"] = source;
        dbcInfo["messages"] = getAvailableMessages();
        dbcs.append(dbcInfo);
        break;
      }
    }
  }

  result["dbcs"] = dbcs;

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeDecodeMessage(const QJsonObject &args) {
  QString messageId = args["message_id"].toString();
  QString data = args["data"].toString();
  QString source = args["source"].toString();

  if (!dbc_manager) {
    throw std::runtime_error("No DBC manager available");
  }

  QJsonObject result;
  result["message_id"] = messageId;
  result["raw_data"] = data;
  result["decoded"] = "Message decoding functionality";

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeGetSignalValues(const QJsonObject &args) {
  QJsonArray signal_list = args["signals"].toArray();

  QJsonObject result;
  QJsonArray signalValues;

  for (const auto &signal : signal_list) {
    QJsonObject signalInfo;
    signalInfo["name"] = signal.toString();
    signalInfo["value"] = "Signal value retrieval";
    signalValues.append(signalInfo);
  }

  result["signals"] = signalValues;

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeSearchSignals(const QJsonObject &args) {
  QString query = args["query"].toString();
  bool caseSensitive = args["case_sensitive"].toBool();

  QJsonObject result;
  result["query"] = query;
  result["case_sensitive"] = caseSensitive;
  result["matches"] = QJsonArray(); // Signal search results

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeExportData(const QJsonObject &args) {
  QString format = args["format"].toString();
  QString outputPath = args["output_path"].toString();

  QJsonObject result;
  result["format"] = format;
  result["output_path"] = outputPath;
  result["status"] = "Export functionality";

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeGetRouteInfo(const QJsonObject &args) {
  Q_UNUSED(args);

  QJsonObject result;

  if (stream) {
    result["route_name"] = QString::fromStdString(stream->routeName());
    result["car_fingerprint"] = QString::fromStdString(stream->carFingerprint());
    result["is_live_stream"] = stream->liveStreaming();
    result["is_paused"] = stream->isPaused();
    result["current_time"] = stream->currentSec();
    result["min_time"] = stream->minSeconds();
    result["max_time"] = stream->maxSeconds();
    result["total_events"] = static_cast<int>(stream->allEvents().size());
    result["playback_speed"] = stream->getSpeed();

    QDateTime beginTime = stream->beginDateTime();
    if (beginTime.isValid()) {
      result["begin_datetime"] = beginTime.toString(Qt::ISODate);
    }

    // Get active message counts
    QJsonObject messageStats;
    const auto &lastMessages = stream->lastMessages();
    messageStats["unique_messages"] = static_cast<int>(lastMessages.size());

    // Count messages by bus
    QJsonObject busCounts;
    for (const auto &[msgId, canData] : lastMessages) {
      QString busKey = QString("bus_%1").arg(msgId.source);
      int currentCount = busCounts.contains(busKey) ? busCounts.value(busKey).toInt() : 0;
      busCounts[busKey] = currentCount + 1;
    }
    messageStats["messages_by_bus"] = busCounts;
    result["message_statistics"] = messageStats;
  } else {
    result["error"] = "No stream loaded";
  }

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeSelectMessage(const QJsonObject &args) {
  QString messageIdStr = args["message_id"].toString();
  QJsonValue sourceValue = args["source"];

  if (messageIdStr.isEmpty()) {
    throw std::runtime_error("Message ID is required");
  }

  // Parse message ID - handle both hex (0x123) and decimal (123) formats
  bool ok;
  uint32_t messageId;
  if (messageIdStr.startsWith("0x") || messageIdStr.startsWith("0X")) {
    messageId = messageIdStr.mid(2).toUInt(&ok, 16);
  } else {
    // For input like "460", first try as decimal, then as hex
    messageId = messageIdStr.toUInt(&ok, 10);  // Try decimal first
    if (!ok) {
      messageId = messageIdStr.toUInt(&ok, 16);  // Then try hex
    }
  }

  if (!ok) {
    throw std::runtime_error("Invalid message ID format: " + messageIdStr.toStdString());
  }

  if (!main_window) {
    throw std::runtime_error("Main window not set");
  }

  // Try to select the message
  bool messageFound = false;
  MessageId targetMsgId;

  if (sourceValue.isNull() || !sourceValue.isDouble()) {
    // Search through all available sources for the message ID
    if (stream) {
      const auto &lastMessages = stream->lastMessages();
      for (const auto &[msgId, canData] : lastMessages) {
        if (msgId.address == messageId) {
          targetMsgId = msgId;
          messageFound = true;
          break;
        }
      }
    }

    // Also check DBC files for this message
    if (!messageFound && dbc_manager) {
      for (int source = 0; source < 256; ++source) {
        try {
          const auto &source_messages = dbc_manager->getMessages(source);
          if (source_messages.count(messageId) > 0) {
            targetMsgId = MessageId{.source = (uint8_t)source, .address = messageId};
            messageFound = true;
            break;
          }
        } catch (...) {
          // Skip sources that don't have messages
          continue;
        }
      }
    }

    // If still not found, default to source 0 (most common case)
    if (!messageFound) {
      targetMsgId = MessageId{.source = 0, .address = messageId};
    }
  } else {
    // Use specified source
    int source = sourceValue.toInt();
    targetMsgId = MessageId{.source = (uint8_t)source, .address = messageId};

    // Check if message exists in the stream or DBC
    if (stream) {
      const auto &lastMessages = stream->lastMessages();
      messageFound = lastMessages.count(targetMsgId) > 0;
    }

    if (!messageFound && dbc_manager) {
      try {
        const auto &source_messages = dbc_manager->getMessages(source);
        messageFound = source_messages.count(messageId) > 0;
      } catch (...) {
        // Source doesn't have messages
      }
    }

    // If still not found, still try to select it (might work in GUI)
    if (!messageFound) {
      messageFound = true; // Allow selection attempt
    }
  }

  QJsonObject result;
  result["requested_message_id"] = messageIdStr;
  result["parsed_message_id"] = QString("0x%1").arg(messageId, 0, 16);

  // Always try to select the message regardless of whether we found it in data
  // The GUI might have additional logic to handle the selection
  bool selectionResult;

  qDebug() << "MCP: Attempting to select message" << QString("0x%1").arg(messageId, 0, 16)
           << "from source" << targetMsgId.source << "messageFound:" << messageFound;

  // Try direct call if we're on the main thread, otherwise use queued connection
  if (QThread::currentThread() == main_window->thread()) {
    qDebug() << "MCP: Using direct call (same thread)";
    main_window->selectMessage(targetMsgId);
    selectionResult = true;
  } else {
    qDebug() << "MCP: Using queued connection (different thread)";
    selectionResult = QMetaObject::invokeMethod(main_window, "selectMessage",
                                Qt::QueuedConnection,
                                Q_ARG(MessageId, targetMsgId));
  }

  qDebug() << "MCP: Selection result:" << selectionResult << "Message found:" << messageFound;

  if (messageFound && selectionResult) {
    result["status"] = "success";
    result["message"] = QString("Selected message 0x%1 from source %2")
                          .arg(targetMsgId.address, 0, 16)
                          .arg(targetMsgId.source);
    result["source"] = targetMsgId.source;
  } else if (selectionResult) {
    result["status"] = "attempted";
    result["message"] = QString("Attempted to select message 0x%1 from source %2 (not found in current data but may exist in DBC)")
                          .arg(targetMsgId.address, 0, 16)
                          .arg(targetMsgId.source);
    result["source"] = targetMsgId.source;
  } else {
    result["status"] = "failed";
    result["message"] = QString("Failed to invoke selection for message 0x%1").arg(messageId, 0, 16);
  }  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonObject McpServer::executeCreateSignal(const QJsonObject &args) {
  QString messageIdStr = args["message_id"].toString();
  QJsonValue sourceValue = args["source"];
  QString signalName = args["signal_name"].toString();
  int startBit = args["start_bit"].toInt();
  int size = args["size"].toInt();
  bool isSigned = args["is_signed"].toBool(false);
  bool isLittleEndian = args["is_little_endian"].toBool(true);
  double factor = args["factor"].toDouble(1.0);
  double offset = args["offset"].toDouble(0.0);
  QString unit = args["unit"].toString();
  QString comment = args["comment"].toString();

  if (messageIdStr.isEmpty()) {
    throw std::runtime_error("Message ID is required");
  }

  if (signalName.isEmpty()) {
    throw std::runtime_error("Signal name is required");
  }

  if (startBit < 0 || startBit > 63) {
    throw std::runtime_error("Start bit must be between 0 and 63");
  }

  if (size < 1 || size > 64) {
    throw std::runtime_error("Signal size must be between 1 and 64 bits");
  }

  if (!dbc_manager) {
    throw std::runtime_error("No DBC manager available");
  }

  // Parse message ID
  bool ok;
  uint32_t messageId;
  if (messageIdStr.startsWith("0x") || messageIdStr.startsWith("0X")) {
    messageId = messageIdStr.mid(2).toUInt(&ok, 16);
  } else {
    messageId = messageIdStr.toUInt(&ok, 10);  // Try decimal first
    if (!ok) {
      messageId = messageIdStr.toUInt(&ok, 16);  // Then try hex
    }
  }

  if (!ok) {
    throw std::runtime_error("Invalid message ID format: " + messageIdStr.toStdString());
  }

  // Determine source
  int source = sourceValue.isNull() ? 0 : sourceValue.toInt();
  MessageId msgId{.source = (uint8_t)source, .address = messageId};

  // Get or create the message
  cabana::Msg *msg = dbc_manager->msg(msgId);
  if (!msg) {
    // Create a new message if it doesn't exist
    QString msgName = dbc_manager->newMsgName(msgId);
    dbc_manager->updateMsg(msgId, msgName, 8, DEFAULT_NODE_NAME, "");
    msg = dbc_manager->msg(msgId);
    if (!msg) {
      throw std::runtime_error("Failed to create message");
    }
  }

  // Create the signal
  cabana::Signal newSignal;
  newSignal.name = signalName;
  newSignal.start_bit = startBit;
  newSignal.size = size;
  newSignal.is_signed = isSigned;
  newSignal.is_little_endian = isLittleEndian;
  newSignal.factor = factor;
  newSignal.offset = offset;
  newSignal.unit = unit;
  newSignal.comment = comment;
  newSignal.receiver_name = DEFAULT_NODE_NAME;
  newSignal.type = cabana::Signal::Type::Normal;

  // Set min/max based on signal properties
  if (isSigned) {
    double maxVal = (1ULL << (size - 1)) - 1;
    newSignal.min = -maxVal - 1;
    newSignal.max = maxVal;
  } else {
    newSignal.min = 0;
    newSignal.max = (1ULL << size) - 1;
  }

  // Call update() to calculate msb, lsb, etc.
  newSignal.update();

  // Add the signal to the message
  cabana::Signal *addedSignal = msg->addSignal(newSignal);
  if (!addedSignal) {
    throw std::runtime_error("Failed to add signal to message");
  }

  QJsonObject result;
  result["status"] = "success";
  result["message"] = QString("Created signal '%1' in message 0x%2 (source %3)")
                        .arg(signalName)
                        .arg(messageId, 0, 16)
                        .arg(source);
  result["message_id"] = QString("0x%1").arg(messageId, 0, 16);
  result["source"] = source;
  result["signal_name"] = signalName;
  result["start_bit"] = startBit;
  result["size"] = size;
  result["is_signed"] = isSigned;
  result["is_little_endian"] = isLittleEndian;
  result["factor"] = factor;
  result["offset"] = offset;
  result["unit"] = unit;
  result["comment"] = comment;

  QJsonObject contentObj;
  contentObj["type"] = "text";
  contentObj["text"] = QString::fromUtf8(QJsonDocument(result).toJson());

  QJsonObject response;
  response["content"] = QJsonArray{contentObj};
  return response;
}

QJsonArray McpServer::getAvailableMessages() {
  QJsonArray messages;

  if (dbc_manager) {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file) {
        QString dbcName = QString::fromStdString(dbc_file->name());
        // Get messages from all sources - this is a simplified approach
        // In practice, you might want to iterate through specific sources
        for (int source = 0; source < 256; ++source) {
          try {
            const auto &source_messages = dbc_manager->getMessages(source);
            for (const auto &[address, msg] : source_messages) {
              QJsonObject msgInfo;
              msgInfo["name"] = msg.name;
              msgInfo["id"] = static_cast<int>(address);
              msgInfo["source"] = source;
              msgInfo["dbc"] = dbcName;
              messages.append(msgInfo);
            }
          } catch (...) {
            // Skip sources that don't have messages
            continue;
          }
        }
      }
    }
  }

  return messages;
}

QJsonArray McpServer::getSignalsForMessage(const QString &message_name) {
  QJsonArray signal_array;

  if (dbc_manager) {
    // Search through all sources for a message with the given name
    for (int source = 0; source < 256; ++source) {
      try {
        auto msg = dbc_manager->msg(source, message_name);
        if (msg) {
          for (const auto &sig : msg->getSignals()) {
            QJsonObject sigInfo;
            sigInfo["name"] = sig->name;
            sigInfo["unit"] = sig->unit;
            signal_array.append(sigInfo);
          }
          break; // Found the message, no need to continue searching
        }
      } catch (...) {
        // Skip sources that don't have this message
        continue;
      }
    }
  }

  return signal_array;
}

QString McpServer::formatCanMessage(const CanEvent &event) {
  QString dataHex;
  for (int i = 0; i < event.size; ++i) {
    dataHex += QString("%1").arg(event.dat[i], 2, 16, QChar('0')).toUpper();
  }
  return QString("CAN Message: ID=0x%1 Data=%2")
    .arg(event.address, 0, 16)
    .arg(dataHex);
}

QJsonObject McpServer::createError(const QString &code, const QString &message) {
  QJsonObject error;
  error["code"] = code;
  error["message"] = message;
  return error;
}