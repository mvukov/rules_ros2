uint64 timestamp
uint64 count
# Shared memory backend can only deal with fixed size messages.
# 3145728 = 3*1024*1024 = 3MiB
char[3145728] data
uint64 data_length
