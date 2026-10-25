#pragma once

// Data files compiled into the core (see cmake/embed_files.cmake).
namespace simpel_embedded {

struct EmbeddedFile {
    const char* name;            // file name without directory
    const unsigned char* data;   // contents; data[size] == 0
    unsigned long size;
};

// data/blocks/*.json, sorted by file name; terminated by a {nullptr} entry.
extern const EmbeddedFile kBlockFiles[];
extern const unsigned long kBlockFiles_count;

} // namespace simpel_embedded
