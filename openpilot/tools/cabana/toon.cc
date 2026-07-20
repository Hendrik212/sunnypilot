#include "tools/cabana/toon.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QStringList>

#include <cmath>
#include <limits>

namespace toon {
namespace {

// --- String quoting ---

bool needsQuoting(const QString &s, QChar delimiter = ',') {
  if (s.isEmpty()) return true;
  if (s == "true" || s == "false" || s == "null") return true;

  // Starts with '-' (could be confused with list marker or negative number)
  if (s.startsWith('-')) return true;

  // Looks like a number: optional minus, digits, optional dot+digits, optional e/E exponent
  static QRegularExpression numRx("^-?(?:0|[1-9]\\d*)(?:\\.\\d+)?(?:[eE][+-]?\\d+)?$");
  if (numRx.match(s).hasMatch()) return true;

  // Leading zeros that look numeric (e.g. "05")
  static QRegularExpression leadingZeroRx("^0\\d");
  if (leadingZeroRx.match(s).hasMatch()) return true;

  // Contains special characters
  for (const QChar &c : s) {
    if (c == ':' || c == '"' || c == '\\' || c == '[' || c == ']' ||
        c == '{' || c == '}' || c == '\n' || c == '\r' || c == '\t') {
      return true;
    }
    if (c == delimiter) return true;
  }

  // Leading or trailing whitespace
  if (s.at(0).isSpace() || s.at(s.size() - 1).isSpace()) return true;

  return false;
}

QString escapeString(const QString &s) {
  QString result;
  result.reserve(s.size() + 8);
  result += '"';
  for (const QChar &c : s) {
    if (c == '\\') result += "\\\\";
    else if (c == '"') result += "\\\"";
    else if (c == '\n') result += "\\n";
    else if (c == '\r') result += "\\r";
    else if (c == '\t') result += "\\t";
    else result += c;
  }
  result += '"';
  return result;
}

QString encodeStringValue(const QString &s, QChar delimiter = ',') {
  if (needsQuoting(s, delimiter)) {
    return escapeString(s);
  }
  return s;
}

// --- Number formatting ---

// Canonical decimal: no exponent, no trailing zeros, -0 → 0
QString formatNumber(double v) {
  if (std::isnan(v) || std::isinf(v)) return "null";
  if (v == 0.0) return "0";  // handles -0

  // Use 'g' format which picks the shorter of fixed/scientific,
  // with enough precision to round-trip doubles. Then strip exponent if present.
  QString s = QString::number(v, 'g', 15);
  // If 'g' chose scientific notation, convert to fixed (TOON spec: no exponent)
  if (s.contains('e') || s.contains('E')) {
    s = QString::number(v, 'f', 10);
    // Strip trailing zeros
    if (s.contains('.')) {
      int i = s.size() - 1;
      while (i > 0 && s[i] == '0') i--;
      if (s[i] == '.') i--;
      s.truncate(i + 1);
    }
  }
  return s;
}

// --- Key encoding ---

QString encodeKey(const QString &key) {
  if (needsQuoting(key, ',')) {
    return escapeString(key);
  }
  return key;
}

// --- Indentation ---

QString indent(int depth, int spaces) {
  return QString(depth * spaces, ' ');
}

// --- Forward declaration ---

void encodeValue(const QJsonValue &value, int depth, int spaces, QStringList &lines,
                 const QString &keyPrefix = QString());

// --- Tabular detection ---

// Check if an array is eligible for tabular encoding:
// all elements are objects with identical sets of keys, all values primitive
struct TabularInfo {
  bool eligible = false;
  QStringList fields;
};

TabularInfo checkTabular(const QJsonArray &arr) {
  TabularInfo info;
  if (arr.isEmpty()) return info;

  // First element must be an object
  if (!arr[0].isObject()) return info;
  QJsonObject first = arr[0].toObject();
  if (first.isEmpty()) return info;

  // Collect field names from first object
  info.fields = first.keys();

  // All values in first object must be primitive
  for (const QString &key : info.fields) {
    QJsonValue v = first[key];
    if (v.isObject() || v.isArray()) return info;
  }

  // All subsequent objects must have the same keys with primitive values
  for (int i = 1; i < arr.size(); ++i) {
    if (!arr[i].isObject()) return info;
    QJsonObject obj = arr[i].toObject();
    if (obj.keys() != info.fields) return info;
    for (const QString &key : info.fields) {
      QJsonValue v = obj[key];
      if (v.isObject() || v.isArray()) return info;
    }
  }

  info.eligible = true;
  return info;
}

// Check if all elements are primitives (for inline array encoding)
bool allPrimitive(const QJsonArray &arr) {
  for (const QJsonValue &v : arr) {
    if (v.isObject() || v.isArray()) return false;
  }
  return true;
}

// Encode a primitive value as an inline string (for tabular rows and inline arrays)
QString encodePrimitive(const QJsonValue &v, QChar delimiter = ',') {
  if (v.isNull() || v.isUndefined()) return "null";
  if (v.isBool()) return v.toBool() ? "true" : "false";
  if (v.isDouble()) return formatNumber(v.toDouble());
  if (v.isString()) return encodeStringValue(v.toString(), delimiter);
  return "null";
}

// --- Array encoding ---

void encodeArray(const QJsonArray &arr, int depth, int spaces, QStringList &lines,
                 const QString &keyPrefix) {
  QString ind = indent(depth, spaces);
  QString header = keyPrefix + QString("[%1]").arg(arr.size());

  if (arr.isEmpty()) {
    lines.append(ind + header + ":");
    return;
  }

  // Inline primitive array
  if (allPrimitive(arr)) {
    QStringList vals;
    for (const QJsonValue &v : arr) {
      vals.append(encodePrimitive(v));
    }
    lines.append(ind + header + ": " + vals.join(","));
    return;
  }

  // Tabular array
  TabularInfo tab = checkTabular(arr);
  if (tab.eligible) {
    // Header with field names
    QStringList escapedFields;
    for (const QString &f : tab.fields) {
      escapedFields.append(f);  // field names in header are unescaped (per spec they're identifiers)
    }
    lines.append(ind + header + "{" + escapedFields.join(",") + "}:");
    QString rowInd = indent(depth + 1, spaces);
    for (const QJsonValue &elem : arr) {
      QJsonObject obj = elem.toObject();
      QStringList vals;
      for (const QString &f : tab.fields) {
        vals.append(encodePrimitive(obj[f]));
      }
      lines.append(rowInd + vals.join(","));
    }
    return;
  }

  // Mixed / list format
  lines.append(ind + header + ":");
  QString itemInd = indent(depth + 1, spaces);
  for (const QJsonValue &elem : arr) {
    if (elem.isObject()) {
      QJsonObject obj = elem.toObject();
      QStringList keys = obj.keys();
      if (keys.isEmpty()) {
        lines.append(itemInd + "-");
      } else {
        // First field on the hyphen line
        QString firstKey = keys[0];
        QJsonValue firstVal = obj[firstKey];
        if (firstVal.isObject() || firstVal.isArray()) {
          // Complex first value: put key on hyphen line, value indented below
          QStringList subLines;
          encodeValue(firstVal, 0, spaces, subLines, encodeKey(firstKey));
          if (!subLines.isEmpty()) {
            lines.append(itemInd + "- " + subLines[0].trimmed());
            QString contInd = indent(depth + 2, spaces);
            for (int i = 1; i < subLines.size(); ++i) {
              lines.append(contInd + subLines[i].trimmed());
            }
          }
        } else {
          lines.append(itemInd + "- " + encodeKey(firstKey) + ": " + encodePrimitive(firstVal));
        }
        // Remaining fields at same indentation as first field
        QString fieldInd = itemInd + "  ";  // align with content after "- "
        for (int k = 1; k < keys.size(); ++k) {
          QStringList subLines;
          encodeValue(obj[keys[k]], depth + 2, spaces, subLines, encodeKey(keys[k]));
          for (const QString &line : subLines) {
            lines.append(line);
          }
        }
      }
    } else if (elem.isArray()) {
      QStringList subLines;
      encodeArray(elem.toArray(), 0, spaces, subLines, "");
      if (!subLines.isEmpty()) {
        lines.append(itemInd + "- " + subLines[0].trimmed());
        for (int i = 1; i < subLines.size(); ++i) {
          lines.append(indent(depth + 2, spaces) + subLines[i].trimmed());
        }
      }
    } else {
      lines.append(itemInd + "- " + encodePrimitive(elem));
    }
  }
}

// --- Main encoder ---

void encodeValue(const QJsonValue &value, int depth, int spaces, QStringList &lines,
                 const QString &keyPrefix) {
  QString ind = indent(depth, spaces);

  if (value.isNull() || value.isUndefined()) {
    if (keyPrefix.isEmpty()) {
      lines.append(ind + "null");
    } else {
      lines.append(ind + keyPrefix + ": null");
    }
    return;
  }

  if (value.isBool()) {
    QString v = value.toBool() ? "true" : "false";
    if (keyPrefix.isEmpty()) {
      lines.append(ind + v);
    } else {
      lines.append(ind + keyPrefix + ": " + v);
    }
    return;
  }

  if (value.isDouble()) {
    QString v = formatNumber(value.toDouble());
    if (keyPrefix.isEmpty()) {
      lines.append(ind + v);
    } else {
      lines.append(ind + keyPrefix + ": " + v);
    }
    return;
  }

  if (value.isString()) {
    QString v = encodeStringValue(value.toString());
    if (keyPrefix.isEmpty()) {
      lines.append(ind + v);
    } else {
      lines.append(ind + keyPrefix + ": " + v);
    }
    return;
  }

  if (value.isArray()) {
    encodeArray(value.toArray(), depth, spaces, lines, keyPrefix.isEmpty() ? "" : keyPrefix);
    return;
  }

  if (value.isObject()) {
    QJsonObject obj = value.toObject();
    if (obj.isEmpty()) {
      // Empty object
      if (!keyPrefix.isEmpty()) {
        lines.append(ind + keyPrefix + ":");
      }
      // Root empty object = empty document (no lines)
      return;
    }
    if (!keyPrefix.isEmpty()) {
      lines.append(ind + keyPrefix + ":");
      depth++;
    }
    QString objInd = indent(depth, spaces);
    QStringList keys = obj.keys();
    for (const QString &key : keys) {
      encodeValue(obj[key], depth, spaces, lines, encodeKey(key));
    }
    return;
  }
}

}  // namespace

QString encode(const QJsonValue &value, int indent) {
  QStringList lines;
  encodeValue(value, 0, indent, lines);
  return lines.join('\n');
}

QString encodeCompact(const QJsonValue &value, int indent) {
  // JSON baseline
  QString json;
  if (value.isArray()) {
    json = QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  } else if (value.isObject()) {
    json = QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
  } else {
    // For primitives, TOON won't be shorter
    return encode(value, indent);
  }

  QString toonStr = encode(value, indent);
  return toonStr.size() < json.size() ? toonStr : json;
}

}  // namespace toon
