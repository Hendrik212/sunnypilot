#include "tools/cabana/mcpserver.h"

#include <QJsonDocument>
#include <QTextStream>
#include <QDebug>
#include <QDateTime>
#include <QMetaObject>
#include <QRegularExpression>
#include <QThread>

#include <cmath>
#include <numeric>
#include <algorithm>

#include "tools/cabana/mainwin.h"
#include "tools/cabana/toon.h"

const QString McpServer::SERVER_NAME = "Cabana MCP Server";
const QString McpServer::SERVER_VERSION = "1.1.0";
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
    {"description", "Decode a CAN message using the loaded DBC. Provide message_id and raw hex data bytes, returns all decoded signal values."},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"message_id", "data"}},
      {"properties", QJsonObject{
        {"message_id", QJsonObject{{"type", "string"}, {"description", "CAN message ID (hex, e.g. '0x3a5')"}}},
        {"data", QJsonObject{{"type", "string"}, {"description", "CAN message data as hex string (e.g. '9C5557430C054169')"}}},
        {"source", QJsonObject{{"type", "integer"}, {"description", "CAN bus source number (default: 0)"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "get_signal_values"},
    {"description", "Get time-series values for one or more signals, resampled onto a uniform time grid. All signals are aligned to the same timestamps. Returns TOON tabular format for compact output."},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"signals"}},
      {"properties", QJsonObject{
        {"signals", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "string"}}}, {"description", "List of signal names to query"}}},
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range (e.g. '60-120', '0-30', 'all'). Default: 'all'"}}},
        {"sample_rate", QJsonObject{{"type", "number"}, {"description", "Seconds per sample (e.g. 0.1 = 10Hz, 0.01 = 100Hz). Default: 0.1"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "search_signals"},
    {"description", "Search for DBC signals by name or regex pattern. Returns signal metadata including parent message, address, bit layout, factor/offset, unit."},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"query"}},
      {"properties", QJsonObject{
        {"query", QJsonObject{{"type", "string"}, {"description", "Search query or regex pattern (e.g. 'STEER', '.*SPEED.*')"}}},
        {"case_sensitive", QJsonObject{{"type", "boolean"}, {"description", "Case sensitive search (default: false)"}}}
      }}
    }}
  },
  QJsonObject{
    {"name", "export_data"},
    {"description", "Export CAN data or DBC definitions. For 'dbc' format returns the DBC file content inline."},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"format"}},
      {"properties", QJsonObject{
        {"format", QJsonObject{{"type", "string"}, {"enum", QJsonArray{"csv", "json", "dbc"}}, {"description", "Export format"}}},
        {"output_path", QJsonObject{{"type", "string"}, {"description", "Output file path (optional, returns inline if omitted)"}}},
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range to export"}}},
        {"messages", QJsonObject{{"type", "array"}, {"items", QJsonObject{{"type", "string"}}}, {"description", "Specific message IDs to export (hex)"}}}
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
  },
  QJsonObject{
    {"name", "analyze_signal_patterns"},
    {"description", "Detect patterns in a signal over time. Methods: 'oscillation' (rolling stddev > threshold, for ping-pong detection), 'threshold' (signal above/below value), 'rate_of_change' (|derivative| > threshold). Returns time ranges where pattern is detected."},
    {"inputSchema", QJsonObject{
      {"type", "object"},
      {"required", QJsonArray{"signal", "method", "threshold"}},
      {"properties", QJsonObject{
        {"signal", QJsonObject{{"type", "string"}, {"description", "Signal name (e.g. 'STEER_ANGLE')"}}},
        {"method", QJsonObject{{"type", "string"}, {"enum", QJsonArray{"oscillation", "threshold", "rate_of_change"}}, {"description", "Detection method"}}},
        {"threshold", QJsonObject{{"type", "number"}, {"description", "Method-specific threshold value"}}},
        {"window", QJsonObject{{"type", "number"}, {"description", "Rolling window in seconds (default: 3.0, used by oscillation)"}}},
        {"min_duration", QJsonObject{{"type", "number"}, {"description", "Minimum passage duration in seconds (default: 2.0)"}}},
        {"time_range", QJsonObject{{"type", "string"}, {"description", "Time range to search (e.g. '60-300' or 'all', default: 'all')"}}},
        {"direction", QJsonObject{{"type", "string"}, {"enum", QJsonArray{"above", "below", "both"}}, {"description", "For threshold method: detect above, below, or both crossings (default: 'above')"}}}
      }}
    }}
  }
};

// ============================================================================
// Server lifecycle & protocol
// ============================================================================

McpServer::McpServer(QObject *parent)
    : QObject(parent), server(nullptr), stream(nullptr), dbc_manager(nullptr), main_window(nullptr) {
}

McpServer::~McpServer() {
  stopServer();
}

bool McpServer::startServer(quint16 port) {
  if (server && server->isListening()) {
    return false;
  }
  server = new QTcpServer(this);
  connect(server, &QTcpServer::newConnection, this, &McpServer::onNewConnection);
  if (!server->listen(QHostAddress::Any, port)) {
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
    for (QTcpSocket *client : clients) {
      if (client->state() != QAbstractSocket::UnconnectedState) {
        client->disconnectFromHost();
        if (client->state() != QAbstractSocket::UnconnectedState) {
          client->waitForDisconnected(1000);
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

void McpServer::setStream(AbstractStream *new_stream) { this->stream = new_stream; }
void McpServer::setDbcManager(DBCManager *new_dbc_manager) { this->dbc_manager = new_dbc_manager; }
void McpServer::setMainWindow(MainWindow *window) { this->main_window = window; }

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
      sendResponse(QJsonObject{{"jsonrpc", "2.0"}, {"error", createError("-32700", "Parse error")}, {"id", QJsonValue::Null}}, client);
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
  if (!id.isNull()) response["id"] = id;
  try {
    if (method == "initialize") response["result"] = handleInitialize(params);
    else if (method == "tools/list") response["result"] = handleToolsList();
    else if (method == "tools/call") response["result"] = handleToolsCall(params);
    else if (method == "resources/list") response["result"] = handleResourcesList();
    else if (method == "resources/read") response["result"] = handleResourcesRead(params);
    else response["error"] = createError("-32601", "Method not found");
  } catch (const std::exception &e) {
    response["error"] = createError("-32603", QString("Internal error: %1").arg(e.what()));
  } catch (...) {
    response["error"] = createError("-32603", "Unknown internal error");
  }
  if (!id.isNull()) sendResponse(response, client);
}

void McpServer::sendResponse(const QJsonObject &response, QTcpSocket *client) {
  if (!client || client->state() != QAbstractSocket::ConnectedState) return;
  QByteArray data = QJsonDocument(response).toJson(QJsonDocument::Compact) + "\n";
  if (client->write(data) == -1) qWarning() << "Failed to write response to client";
  client->flush();
}

void McpServer::sendNotification(const QJsonObject &notification, QTcpSocket *client) {
  sendResponse(notification, client);
}

QJsonObject McpServer::handleInitialize(const QJsonObject &params) {
  Q_UNUSED(params);
  return QJsonObject{
    {"protocolVersion", "2024-11-05"},
    {"capabilities", QJsonObject{{"tools", QJsonObject{}}, {"resources", QJsonObject{}}}},
    {"serverInfo", QJsonObject{{"name", SERVER_NAME}, {"version", SERVER_VERSION}}}
  };
}

QJsonObject McpServer::handleToolsList() {
  return QJsonObject{{"tools", SUPPORTED_TOOLS}};
}

QJsonObject McpServer::handleToolsCall(const QJsonObject &params) {
  QString toolName = params["name"].toString();
  QJsonObject args = params["arguments"].toObject();
  if (toolName == "analyze_can_messages") return executeAnalyzeCanMessages(args);
  if (toolName == "get_dbc_info") return executeGetDbcInfo(args);
  if (toolName == "decode_message") return executeDecodeMessage(args);
  if (toolName == "get_signal_values") return executeGetSignalValues(args);
  if (toolName == "search_signals") return executeSearchSignals(args);
  if (toolName == "export_data") return executeExportData(args);
  if (toolName == "get_route_info") return executeGetRouteInfo(args);
  if (toolName == "select_message") return executeSelectMessage(args);
  if (toolName == "create_signal") return executeCreateSignal(args);
  if (toolName == "analyze_signal_patterns") return executeAnalyzeSignalPatterns(args);
  throw std::runtime_error("Unknown tool: " + toolName.toStdString());
}

// ============================================================================
// Helper methods
// ============================================================================

QJsonObject McpServer::makeTextResponse(const QJsonValue &result) {
  QJsonObject contentObj;
  contentObj["type"] = "text";
  if (result.isString()) {
    contentObj["text"] = result.toString();
  } else if (result.isObject()) {
    contentObj["text"] = QString::fromUtf8(QJsonDocument(result.toObject()).toJson(QJsonDocument::Compact));
  } else if (result.isArray()) {
    contentObj["text"] = QString::fromUtf8(QJsonDocument(result.toArray()).toJson(QJsonDocument::Compact));
  }
  return QJsonObject{{"content", QJsonArray{contentObj}}};
}

uint32_t McpServer::parseHexId(const QString &str) const {
  bool ok;
  uint32_t id;
  if (str.startsWith("0x") || str.startsWith("0X")) {
    id = str.mid(2).toUInt(&ok, 16);
  } else {
    id = str.toUInt(&ok, 10);
    if (!ok) id = str.toUInt(&ok, 16);
  }
  if (!ok) throw std::runtime_error("Invalid message ID: " + str.toStdString());
  return id;
}

std::pair<double, double> McpServer::parseTimeRange(const QString &str) const {
  if (str.isEmpty() || str == "all") {
    double tMin = stream ? stream->minSeconds() : 0;
    double tMax = stream ? stream->maxSeconds() : 0;
    return {tMin, tMax};
  }
  // Format: "start-end" e.g. "60.5-135.2"
  int dashIdx = str.indexOf('-', 1);  // skip potential leading minus
  if (dashIdx > 0) {
    bool ok1, ok2;
    double start = str.left(dashIdx).toDouble(&ok1);
    double end = str.mid(dashIdx + 1).toDouble(&ok2);
    if (ok1 && ok2) return {start, end};
  }
  throw std::runtime_error("Invalid time_range format: " + str.toStdString() + " (use 'all' or 'start-end')");
}

McpServer::ResolvedSignal McpServer::resolveSignal(const QString &name) const {
  if (!dbc_manager) return {};
  std::string stdName = name.toStdString();
  for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
    if (!dbc_file) continue;
    for (const auto &[address, msg] : dbc_file->getMessages()) {
      for (const auto *sig : msg.getSignals()) {
        if (sig->name == stdName) {
          // Find the actual source by checking which bus has this address in the stream
          uint8_t src = 0;
          if (stream) {
            for (const auto &[msgId, canData] : stream->lastMessages()) {
              if (msgId.address == address) {
                src = msgId.source;
                break;
              }
            }
          }
          return {sig, MessageId{.source = src, .address = address}};
        }
      }
    }
  }
  return {};
}

QJsonObject McpServer::createError(const QString &code, const QString &message) {
  return QJsonObject{{"code", code}, {"message", message}};
}

// ============================================================================
// Resources
// ============================================================================

QJsonObject McpServer::handleResourcesList() {
  QJsonArray resources;
  if (dbc_manager) {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file) {
        QString name = QString::fromStdString(dbc_file->name());
        resources.append(QJsonObject{{"uri", "dbc://" + name}, {"name", "DBC: " + name}, {"mimeType", "application/x-dbc"}});
      }
    }
  }
  if (stream) {
    resources.append(QJsonObject{{"uri", "stream://current"}, {"name", "Current CAN Stream"}, {"mimeType", "application/json"}});
  }
  return QJsonObject{{"resources", resources}};
}

QJsonObject McpServer::handleResourcesRead(const QJsonObject &params) {
  QString uri = params["uri"].toString();
  if (uri.startsWith("dbc://")) {
    QString dbcName = uri.mid(6);
    if (dbc_manager) {
      for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
        if (dbc_file && QString::fromStdString(dbc_file->name()) == dbcName) {
          QJsonObject dbcJson{{"name", dbcName}, {"messages", getAvailableMessages()}};
          return QJsonObject{{"contents", QJsonArray{QJsonObject{
            {"uri", uri}, {"mimeType", "application/json"},
            {"text", QString::fromUtf8(QJsonDocument(dbcJson).toJson())}
          }}}};
        }
      }
    }
  } else if (uri == "stream://current" && stream) {
    QJsonObject info{{"type", "CAN Stream"}, {"active", stream->liveStreaming()}, {"messages_available", getAvailableMessages()}};
    return QJsonObject{{"contents", QJsonArray{QJsonObject{
      {"uri", uri}, {"mimeType", "application/json"},
      {"text", QString::fromUtf8(QJsonDocument(info).toJson())}
    }}}};
  }
  throw std::runtime_error("Resource not found: " + uri.toStdString());
}

// ============================================================================
// Tool: analyze_can_messages (preserved from original)
// ============================================================================

QJsonObject McpServer::executeAnalyzeCanMessages(const QJsonObject &args) {
  if (!stream) throw std::runtime_error("No stream available");

  QJsonArray busFilter = args["bus_filter"].toArray();
  QJsonObject result;
  result["time_range_requested"] = args["time_range"].toString("all");
  result["total_events"] = static_cast<int>(stream->allEvents().size());
  result["route_name"] = QString::fromStdString(stream->routeName());

  const auto &lastMessages = stream->lastMessages();
  QJsonArray activeMessages;
  for (const auto &[msgId, canData] : lastMessages) {
    if (!busFilter.isEmpty()) {
      bool match = false;
      for (const auto &v : busFilter) { if (v.toInt() == msgId.source) { match = true; break; } }
      if (!match) continue;
    }
    QJsonObject info;
    info["message_id"] = QString("0x%1").arg(msgId.address, 0, 16);
    info["source"] = msgId.source;
    info["count"] = static_cast<int>(canData.count);
    info["frequency"] = canData.freq;
    info["last_seen"] = canData.ts;
    info["data_length"] = static_cast<int>(canData.dat.size());
    QString hex;
    for (int i = 0; i < std::min(8, (int)canData.dat.size()); ++i)
      hex += QString("%1").arg(canData.dat[i], 2, 16, QChar('0')).toUpper();
    info["last_data"] = hex;
    activeMessages.append(info);
  }
  result["active_messages"] = activeMessages;
  result["total_unique_messages"] = activeMessages.size();
  return makeTextResponse(result);
}

// ============================================================================
// Tool: get_dbc_info (preserved from original)
// ============================================================================

QJsonObject McpServer::executeGetDbcInfo(const QJsonObject &args) {
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");
  QString source = args["source"].toString();
  QJsonArray dbcs;
  for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
    if (!dbc_file) continue;
    QString name = QString::fromStdString(dbc_file->name());
    if (!source.isEmpty() && name != source) continue;
    dbcs.append(QJsonObject{{"name", name}, {"messages", getAvailableMessages()}});
    if (!source.isEmpty()) break;
  }
  return makeTextResponse(QJsonObject{{"dbcs", dbcs}});
}

// ============================================================================
// Tool: decode_message (IMPLEMENTED)
// ============================================================================

QJsonObject McpServer::executeDecodeMessage(const QJsonObject &args) {
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");

  uint32_t address = parseHexId(args["message_id"].toString());
  QString dataStr = args["data"].toString().remove(' ');
  int src = args["source"].toInt(0);

  // Parse hex data string to bytes
  if (dataStr.size() % 2 != 0) throw std::runtime_error("Hex data must have even number of characters");
  std::vector<uint8_t> data;
  for (int i = 0; i < dataStr.size(); i += 2) {
    bool ok;
    uint8_t byte = dataStr.mid(i, 2).toUInt(&ok, 16);
    if (!ok) throw std::runtime_error("Invalid hex data at position " + std::to_string(i));
    data.push_back(byte);
  }

  MessageId msgId{.source = (uint8_t)src, .address = address};
  cabana::Msg *msg = dbc_manager->msg(msgId);

  QJsonObject result;
  result["message_id"] = QString("0x%1").arg(address, 0, 16);
  result["source"] = src;
  result["data_hex"] = dataStr.toUpper();
  result["data_length"] = (int)data.size();

  if (msg) {
    result["message_name"] = QString::fromStdString(msg->name);
    QJsonArray sigArray;
    for (const auto *sig : msg->getSignals()) {
      double val = 0;
      bool ok = sig->getValue(data.data(), data.size(), &val);
      QJsonObject sigObj;
      sigObj["name"] = QString::fromStdString(sig->name);
      sigObj["value"] = ok ? val : 0.0;
      sigObj["formatted"] = QString::fromStdString(sig->formatValue(val));
      sigObj["unit"] = QString::fromStdString(sig->unit);
      sigObj["start_bit"] = sig->start_bit;
      sigObj["size"] = sig->size;
      sigObj["factor"] = sig->factor;
      sigObj["offset"] = sig->offset;
      sigArray.append(sigObj);
    }
    result["signals"] = sigArray;
  } else {
    result["message_name"] = "unknown";
    result["signals"] = QJsonArray();
    result["note"] = "No DBC definition found for this message ID";
  }
  return makeTextResponse(result);
}

// ============================================================================
// Tool: get_signal_values (IMPLEMENTED - resampling + TOON)
// ============================================================================

QJsonObject McpServer::executeGetSignalValues(const QJsonObject &args) {
  if (!stream) throw std::runtime_error("No stream available");
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");

  QJsonArray signalNames = args["signals"].toArray();
  if (signalNames.isEmpty()) throw std::runtime_error("At least one signal name required");

  double sampleRate = args["sample_rate"].toDouble(0.1);
  if (sampleRate <= 0) throw std::runtime_error("sample_rate must be positive");

  auto [tStart, tEnd] = parseTimeRange(args["time_range"].toString("all"));
  if (tEnd <= tStart) throw std::runtime_error("Invalid time range");

  // Resolve all signals
  struct SignalInfo {
    ResolvedSignal resolved;
    QString name;
  };
  std::vector<SignalInfo> infos;
  for (const auto &v : signalNames) {
    QString name = v.toString();
    auto resolved = resolveSignal(name);
    if (!resolved.sig) throw std::runtime_error("Signal not found: " + name.toStdString());
    infos.push_back({resolved, name});
  }

  // Build uniform time grid
  int numSamples = (int)std::ceil((tEnd - tStart) / sampleRate) + 1;
  if (numSamples > 100000) throw std::runtime_error("Too many samples (max 100000). Increase sample_rate or reduce time range.");

  std::vector<double> timeGrid(numSamples);
  for (int i = 0; i < numSamples; ++i) {
    timeGrid[i] = std::round((tStart + i * sampleRate) * 1000.0) / 1000.0;
    if (timeGrid[i] > tEnd) { timeGrid.resize(i); numSamples = i; break; }
  }

  // For each signal, get events and resample onto the grid (nearest value)
  // Build as TOON-compatible tabular data: array of objects {ts, sig1, sig2, ...}
  QJsonArray rows;
  // Pre-fetch decoded values for each signal
  struct SignalTimeSeries {
    std::vector<double> timestamps;
    std::vector<double> values;
  };
  std::vector<SignalTimeSeries> allSeries(infos.size());

  for (size_t s = 0; s < infos.size(); ++s) {
    auto &info = infos[s];
    auto range = std::make_optional(std::make_pair(tStart, tEnd));
    auto [begin, end] = stream->eventsInRange(info.resolved.msg_id, range);
    auto &series = allSeries[s];
    for (auto it = begin; it != end; ++it) {
      const CanEvent *ev = *it;
      double ts = stream->toSeconds(ev->mono_time);
      double val = 0;
      info.resolved.sig->getValue(ev->dat, ev->size, &val);
      series.timestamps.push_back(ts);
      series.values.push_back(val);
    }
  }

  // Resample: for each grid point, find nearest event value per signal
  for (int i = 0; i < numSamples; ++i) {
    double t = timeGrid[i];
    QJsonObject row;
    row["ts"] = std::round(t * 1000.0) / 1000.0;

    for (size_t s = 0; s < infos.size(); ++s) {
      auto &series = allSeries[s];
      double val = 0;
      if (!series.timestamps.empty()) {
        // Binary search for nearest timestamp
        auto it = std::lower_bound(series.timestamps.begin(), series.timestamps.end(), t);
        if (it == series.timestamps.end()) {
          val = series.values.back();
        } else if (it == series.timestamps.begin()) {
          val = series.values.front();
        } else {
          auto prev = std::prev(it);
          size_t idx = (t - *prev <= *it - t) ? std::distance(series.timestamps.begin(), prev)
                                                : std::distance(series.timestamps.begin(), it);
          val = series.values[idx];
        }
      }
      // Use signal name as column key
      row[infos[s].name] = std::round(val * 10000.0) / 10000.0;
    }
    rows.append(row);
  }

  // Encode as TOON (tabular) or JSON, whichever is smaller
  QString encoded = toon::encodeCompact(QJsonValue(rows));

  QJsonObject result;
  result["time_range"] = QString("%1-%2").arg(tStart).arg(tEnd);
  result["sample_rate"] = sampleRate;
  result["num_samples"] = numSamples;
  result["signals"] = signalNames;
  result["encoding"] = encoded.startsWith('[') ? "toon" : "json";

  QJsonObject contentObj;
  contentObj["type"] = "text";
  // Return metadata as JSON, then the actual data separately
  QString text = QString::fromUtf8(QJsonDocument(result).toJson(QJsonDocument::Compact));
  text += "\n\n--- DATA ---\n" + encoded;
  contentObj["text"] = text;

  return QJsonObject{{"content", QJsonArray{contentObj}}};
}

// ============================================================================
// Tool: search_signals (IMPLEMENTED)
// ============================================================================

QJsonObject McpServer::executeSearchSignals(const QJsonObject &args) {
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");

  QString query = args["query"].toString();
  bool caseSensitive = args["case_sensitive"].toBool(false);

  QRegularExpression rx(query, caseSensitive ? QRegularExpression::NoPatternOption
                                             : QRegularExpression::CaseInsensitiveOption);
  bool useRegex = rx.isValid();

  QJsonArray matches;
  for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
    if (!dbc_file) continue;
    for (const auto &[address, msg] : dbc_file->getMessages()) {
      // Find actual source from stream data
      uint8_t src = 0;
      if (stream) {
        for (const auto &[msgId, canData] : stream->lastMessages()) {
          if (msgId.address == address) { src = msgId.source; break; }
        }
      }

      for (const auto *sig : msg.getSignals()) {
        QString sigName = QString::fromStdString(sig->name);
        bool matched = false;
        if (useRegex) {
          matched = rx.match(sigName).hasMatch();
        } else {
          matched = sigName.contains(query, caseSensitive ? Qt::CaseSensitive : Qt::CaseInsensitive);
        }
        if (!matched) continue;

        QJsonObject info;
        info["signal_name"] = sigName;
        info["message_name"] = QString::fromStdString(msg.name);
        info["message_id"] = QString("0x%1").arg(address, 0, 16);
        info["source"] = src;
        info["start_bit"] = sig->start_bit;
        info["size"] = sig->size;
        info["is_signed"] = sig->is_signed;
        info["is_little_endian"] = sig->is_little_endian;
        info["factor"] = sig->factor;
        info["offset"] = sig->offset;
        info["unit"] = QString::fromStdString(sig->unit);
        if (!sig->comment.empty())
          info["comment"] = QString::fromStdString(sig->comment);
        matches.append(info);
      }
    }
  }

  QJsonObject result;
  result["query"] = query;
  result["case_sensitive"] = caseSensitive;
  result["total_matches"] = matches.size();
  result["matches"] = matches;
  return makeTextResponse(result);
}

// ============================================================================
// Tool: export_data (IMPLEMENTED)
// ============================================================================

QJsonObject McpServer::executeExportData(const QJsonObject &args) {
  QString format = args["format"].toString();

  if (format == "dbc") {
    if (!dbc_manager) throw std::runtime_error("No DBC manager available");
    QJsonArray dbcContents;
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (!dbc_file) continue;
      QJsonObject entry;
      entry["name"] = QString::fromStdString(dbc_file->name());
      entry["content"] = QString::fromStdString(dbc_file->generateDBC());
      dbcContents.append(entry);
    }
    return makeTextResponse(QJsonObject{{"format", "dbc"}, {"files", dbcContents}});
  }

  if (format == "json") {
    if (!stream) throw std::runtime_error("No stream available");
    auto [tStart, tEnd] = parseTimeRange(args["time_range"].toString("all"));
    QJsonArray messagesFilter = args["messages"].toArray();

    QJsonArray data;
    const auto &lastMessages = stream->lastMessages();
    int count = 0;
    for (const auto &[msgId, canData] : lastMessages) {
      if (!messagesFilter.isEmpty()) {
        bool match = false;
        for (const auto &v : messagesFilter) {
          uint32_t filterId = parseHexId(v.toString());
          if (filterId == msgId.address) { match = true; break; }
        }
        if (!match) continue;
      }
      auto range = std::make_optional(std::make_pair(tStart, tEnd));
      auto [begin, end] = stream->eventsInRange(msgId, range);
      for (auto it = begin; it != end && count < 50000; ++it, ++count) {
        const CanEvent *ev = *it;
        QJsonObject row;
        row["ts"] = stream->toSeconds(ev->mono_time);
        row["id"] = QString("0x%1").arg(ev->address, 0, 16);
        row["bus"] = ev->src;
        QString hex;
        for (int i = 0; i < ev->size; ++i)
          hex += QString("%1").arg(ev->dat[i], 2, 16, QChar('0')).toUpper();
        row["data"] = hex;
        data.append(row);
      }
    }
    return makeTextResponse(QJsonObject{{"format", "json"}, {"events", data}, {"count", count}});
  }

  if (format == "csv") {
    if (!stream) throw std::runtime_error("No stream available");
    auto [tStart, tEnd] = parseTimeRange(args["time_range"].toString("all"));
    QString outputPath = args["output_path"].toString();

    // Build CSV inline
    QString csv = "time,id,bus,data\n";
    const auto &lastMessages = stream->lastMessages();
    int count = 0;
    for (const auto &[msgId, canData] : lastMessages) {
      auto range = std::make_optional(std::make_pair(tStart, tEnd));
      auto [begin, end] = stream->eventsInRange(msgId, range);
      for (auto it = begin; it != end && count < 50000; ++it, ++count) {
        const CanEvent *ev = *it;
        QString hex;
        for (int i = 0; i < ev->size; ++i)
          hex += QString("%1").arg(ev->dat[i], 2, 16, QChar('0')).toUpper();
        csv += QString("%1,0x%2,%3,%4\n")
          .arg(stream->toSeconds(ev->mono_time), 0, 'f', 6)
          .arg(ev->address, 0, 16)
          .arg(ev->src)
          .arg(hex);
      }
    }

    if (!outputPath.isEmpty()) {
      QFile file(outputPath);
      if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot write to: " + outputPath.toStdString());
      file.write(csv.toUtf8());
      return makeTextResponse(QJsonObject{{"format", "csv"}, {"output_path", outputPath}, {"count", count}});
    }
    return makeTextResponse(QJsonObject{{"format", "csv"}, {"count", count}, {"content", csv}});
  }

  throw std::runtime_error("Unsupported format: " + format.toStdString());
}

// ============================================================================
// Tool: get_route_info (preserved from original)
// ============================================================================

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
    if (beginTime.isValid()) result["begin_datetime"] = beginTime.toString(Qt::ISODate);
    QJsonObject messageStats;
    const auto &lastMessages = stream->lastMessages();
    messageStats["unique_messages"] = static_cast<int>(lastMessages.size());
    QJsonObject busCounts;
    for (const auto &[msgId, canData] : lastMessages) {
      QString busKey = QString("bus_%1").arg(msgId.source);
      busCounts[busKey] = busCounts.value(busKey).toInt() + 1;
    }
    messageStats["messages_by_bus"] = busCounts;
    result["message_statistics"] = messageStats;
  } else {
    result["error"] = "No stream loaded";
  }
  return makeTextResponse(result);
}

// ============================================================================
// Tool: select_message (preserved from original)
// ============================================================================

QJsonObject McpServer::executeSelectMessage(const QJsonObject &args) {
  QString messageIdStr = args["message_id"].toString();
  QJsonValue sourceValue = args["source"];
  if (messageIdStr.isEmpty()) throw std::runtime_error("Message ID is required");
  if (!main_window) throw std::runtime_error("Main window not set");

  uint32_t messageId = parseHexId(messageIdStr);
  bool messageFound = false;
  MessageId targetMsgId;

  if (sourceValue.isNull() || !sourceValue.isDouble()) {
    if (stream) {
      for (const auto &[msgId, canData] : stream->lastMessages()) {
        if (msgId.address == messageId) { targetMsgId = msgId; messageFound = true; break; }
      }
    }
    if (!messageFound && dbc_manager) {
      for (int source = 0; source < 256; ++source) {
        try {
          if (dbc_manager->getMessages(source).count(messageId) > 0) {
            targetMsgId = MessageId{.source = (uint8_t)source, .address = messageId};
            messageFound = true; break;
          }
        } catch (...) {}
      }
    }
    if (!messageFound) targetMsgId = MessageId{.source = 0, .address = messageId};
  } else {
    int source = sourceValue.toInt();
    targetMsgId = MessageId{.source = (uint8_t)source, .address = messageId};
  }

  bool selectionResult;
  if (QThread::currentThread() == main_window->thread()) {
    main_window->selectMessage(targetMsgId);
    selectionResult = true;
  } else {
    selectionResult = QMetaObject::invokeMethod(main_window, "selectMessage",
                                Qt::QueuedConnection, Q_ARG(MessageId, targetMsgId));
  }

  QJsonObject result;
  result["requested_message_id"] = messageIdStr;
  result["parsed_message_id"] = QString("0x%1").arg(messageId, 0, 16);
  result["source"] = targetMsgId.source;
  result["status"] = selectionResult ? "success" : "failed";
  return makeTextResponse(result);
}

// ============================================================================
// Tool: create_signal (preserved from original)
// ============================================================================

QJsonObject McpServer::executeCreateSignal(const QJsonObject &args) {
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");
  QString messageIdStr = args["message_id"].toString();
  QString signalName = args["signal_name"].toString();
  if (messageIdStr.isEmpty()) throw std::runtime_error("Message ID is required");
  if (signalName.isEmpty()) throw std::runtime_error("Signal name is required");

  int startBit = args["start_bit"].toInt();
  int size = args["size"].toInt();
  if (startBit < 0 || startBit > 63) throw std::runtime_error("Start bit must be 0-63");
  if (size < 1 || size > 64) throw std::runtime_error("Size must be 1-64");

  uint32_t messageId = parseHexId(messageIdStr);
  int source = args["source"].toInt(0);
  MessageId msgId{.source = (uint8_t)source, .address = messageId};

  cabana::Msg *msg = dbc_manager->msg(msgId);
  if (!msg) {
    std::string msgName = dbc_manager->newMsgName(msgId);
    dbc_manager->updateMsg(msgId, msgName, 8, DEFAULT_NODE_NAME, "");
    msg = dbc_manager->msg(msgId);
    if (!msg) throw std::runtime_error("Failed to create message");
  }

  cabana::Signal newSignal;
  newSignal.name = signalName.toStdString();
  newSignal.start_bit = startBit;
  newSignal.size = size;
  newSignal.is_signed = args["is_signed"].toBool(false);
  newSignal.is_little_endian = args["is_little_endian"].toBool(true);
  newSignal.factor = args["factor"].toDouble(1.0);
  newSignal.offset = args["offset"].toDouble(0.0);
  newSignal.unit = args["unit"].toString().toStdString();
  newSignal.comment = args["comment"].toString().toStdString();
  newSignal.receiver_name = DEFAULT_NODE_NAME;
  newSignal.type = cabana::Signal::Type::Normal;
  if (newSignal.is_signed) {
    double maxVal = (1ULL << (size - 1)) - 1;
    newSignal.min = -maxVal - 1;
    newSignal.max = maxVal;
  } else {
    newSignal.min = 0;
    newSignal.max = (1ULL << size) - 1;
  }
  newSignal.update();

  cabana::Signal *added = msg->addSignal(newSignal);
  if (!added) throw std::runtime_error("Failed to add signal");

  QJsonObject result;
  result["status"] = "success";
  result["message_id"] = QString("0x%1").arg(messageId, 0, 16);
  result["source"] = source;
  result["signal_name"] = signalName;
  result["start_bit"] = startBit;
  result["size"] = size;
  return makeTextResponse(result);
}

// ============================================================================
// Tool: analyze_signal_patterns (NEW)
// ============================================================================

QJsonObject McpServer::executeAnalyzeSignalPatterns(const QJsonObject &args) {
  if (!stream) throw std::runtime_error("No stream available");
  if (!dbc_manager) throw std::runtime_error("No DBC manager available");

  QString signalName = args["signal"].toString();
  QString method = args["method"].toString();
  double threshold = args["threshold"].toDouble();
  double window = args["window"].toDouble(3.0);
  double minDuration = args["min_duration"].toDouble(2.0);
  QString direction = args["direction"].toString("above");

  auto resolved = resolveSignal(signalName);
  if (!resolved.sig) throw std::runtime_error("Signal not found: " + signalName.toStdString());

  auto [tStart, tEnd] = parseTimeRange(args["time_range"].toString("all"));

  // Decode all values at full resolution
  auto range = std::make_optional(std::make_pair(tStart, tEnd));
  auto [begin, end] = stream->eventsInRange(resolved.msg_id, range);

  std::vector<double> timestamps;
  std::vector<double> values;
  for (auto it = begin; it != end; ++it) {
    const CanEvent *ev = *it;
    double val = 0;
    resolved.sig->getValue(ev->dat, ev->size, &val);
    timestamps.push_back(stream->toSeconds(ev->mono_time));
    values.push_back(val);
  }

  if (values.empty()) throw std::runtime_error("No data for signal in time range");

  // Compute the metric series based on method
  std::vector<double> metric(values.size(), 0.0);

  if (method == "oscillation") {
    // Rolling standard deviation
    for (size_t i = 0; i < values.size(); ++i) {
      double tCenter = timestamps[i];
      double halfWin = window / 2.0;
      // Collect values within the window
      double sum = 0, sumSq = 0;
      int count = 0;
      // Search backward and forward from i
      for (size_t j = i; j < values.size() && timestamps[j] <= tCenter + halfWin; ++j) {
        sum += values[j]; sumSq += values[j] * values[j]; count++;
      }
      for (size_t j = i; j > 0; --j) {
        if (timestamps[j - 1] < tCenter - halfWin) break;
        sum += values[j - 1]; sumSq += values[j - 1] * values[j - 1]; count++;
      }
      if (count > 1) {
        double mean = sum / count;
        double variance = (sumSq / count) - (mean * mean);
        metric[i] = std::sqrt(std::max(0.0, variance));
      }
    }
  } else if (method == "threshold") {
    for (size_t i = 0; i < values.size(); ++i) {
      if (direction == "above") metric[i] = values[i] > threshold ? 1.0 : 0.0;
      else if (direction == "below") metric[i] = values[i] < threshold ? 1.0 : 0.0;
      else metric[i] = (values[i] > threshold || values[i] < -threshold) ? 1.0 : 0.0;
    }
    threshold = 0.5;  // binary metric, threshold for passage detection
  } else if (method == "rate_of_change") {
    for (size_t i = 1; i < values.size(); ++i) {
      double dt = timestamps[i] - timestamps[i - 1];
      if (dt > 0) metric[i] = std::abs((values[i] - values[i - 1]) / dt);
    }
  } else {
    throw std::runtime_error("Unknown method: " + method.toStdString());
  }

  // Find passages where metric exceeds threshold
  struct Passage {
    double start, end;
    double peakMetric, sumVal;
    int count;
  };
  std::vector<Passage> passages;
  bool inPassage = false;
  Passage current{};

  for (size_t i = 0; i < metric.size(); ++i) {
    bool above = metric[i] >= threshold;
    if (above && !inPassage) {
      inPassage = true;
      current = {timestamps[i], timestamps[i], metric[i], values[i], 1};
    } else if (above && inPassage) {
      current.end = timestamps[i];
      current.peakMetric = std::max(current.peakMetric, metric[i]);
      current.sumVal += values[i];
      current.count++;
    } else if (!above && inPassage) {
      inPassage = false;
      if (current.end - current.start >= minDuration) {
        passages.push_back(current);
      }
    }
  }
  if (inPassage && current.end - current.start >= minDuration) {
    passages.push_back(current);
  }

  // Build result
  QJsonArray passageArray;
  for (const auto &p : passages) {
    QJsonObject obj;
    obj["start"] = QString::number(p.start, 'f', 2).toDouble();
    obj["end"] = QString::number(p.end, 'f', 2).toDouble();
    obj["duration"] = QString::number(p.end - p.start, 'f', 2).toDouble();
    obj["peak_metric"] = QString::number(p.peakMetric, 'f', 4).toDouble();
    obj["mean_value"] = QString::number(p.sumVal / p.count, 'f', 4).toDouble();
    obj["sample_count"] = p.count;
    passageArray.append(obj);
  }

  QJsonObject result;
  result["signal"] = signalName;
  result["method"] = method;
  result["threshold"] = threshold;
  result["window"] = window;
  result["min_duration"] = minDuration;
  result["time_range"] = QString("%1-%2").arg(tStart).arg(tEnd);
  result["total_samples"] = (int)values.size();
  result["passages_found"] = passageArray.size();
  result["passages"] = passageArray;
  return makeTextResponse(result);
}

// ============================================================================
// Utility functions
// ============================================================================

QJsonArray McpServer::getAvailableMessages() {
  QJsonArray messages;
  if (dbc_manager) {
    for (const auto &dbc_file : dbc_manager->allDBCFiles()) {
      if (dbc_file) {
        QString dbcName = QString::fromStdString(dbc_file->name());
        for (int source = 0; source < 256; ++source) {
          try {
            const auto &source_messages = dbc_manager->getMessages(source);
            for (const auto &[address, msg] : source_messages) {
              messages.append(QJsonObject{
                {"name", QString::fromStdString(msg.name)},
                {"id", static_cast<int>(address)},
                {"source", source},
                {"dbc", dbcName}
              });
            }
          } catch (...) { continue; }
        }
      }
    }
  }
  return messages;
}

QJsonArray McpServer::getSignalsForMessage(const QString &message_name) {
  QJsonArray signal_array;
  if (dbc_manager) {
    for (int source = 0; source < 256; ++source) {
      try {
        auto msg = dbc_manager->msg(source, message_name.toStdString());
        if (msg) {
          for (const auto &sig : msg->getSignals()) {
            signal_array.append(QJsonObject{
              {"name", QString::fromStdString(sig->name)},
              {"unit", QString::fromStdString(sig->unit)}
            });
          }
          break;
        }
      } catch (...) { continue; }
    }
  }
  return signal_array;
}

QString McpServer::formatCanMessage(const CanEvent &event) {
  QString hex;
  for (int i = 0; i < event.size; ++i)
    hex += QString("%1").arg(event.dat[i], 2, 16, QChar('0')).toUpper();
  return QString("CAN Message: ID=0x%1 Data=%2").arg(event.address, 0, 16).arg(hex);
}
