## Programm1
Thread 1 read stdin, validate input and then write it to the shared memory (in loop)

Thread 2 read shared memory, print result and calculate "checksum". Also send "checksum" to Programm2 through socket (in loop)

Threads must be synchronized (not via global variable). Thread 2 must not constantly check shared memory

## Programm2
Read from socket and check input