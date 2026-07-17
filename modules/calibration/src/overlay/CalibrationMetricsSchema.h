#pragma once

#include <cstddef>
#include <iosfwd>

namespace Metrics {

// One replay-CSV column: header name plus a writer that appends the value
// for the current tick snapshot.
struct CsvField
{
	const char* name;
	void (*writer)(std::ostream& s);
};

// Dev-build replay CSV schema. Returns nullptr with count 0 in release
// builds, where the replay CSV is compiled out.
const CsvField* CsvSchemaFields(std::size_t& count);

// Clear the per-tick replay snapshots once a row has been written.
void ResetTickReplaySnapshots();

} // namespace Metrics
