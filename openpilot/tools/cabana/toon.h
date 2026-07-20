#pragma once

#include <QJsonValue>
#include <QString>

namespace toon {

// Encode a QJsonValue (object, array, or primitive) to TOON format string.
// TOON is a compact, human-readable encoding of the JSON data model optimized
// for LLM token efficiency. Tabular arrays (uniform objects with primitive values)
// are encoded as CSV-style rows with a single header, reducing tokens ~40%.
//
// Spec: https://toonformat.dev (v3.0)
QString encode(const QJsonValue &value, int indent = 2);

// Encode and pick whichever format (TOON or compact JSON) is shorter.
QString encodeCompact(const QJsonValue &value, int indent = 2);

}  // namespace toon
