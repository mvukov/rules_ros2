uint64 timestamp
uint64 count
# Shared memory backend can only deal with fixed size messages.
# For reference, FHD image: 3 * 1080 * 1920 = 6220800 bytes.
char[6220800] data
uint64 data_length
