#ifndef PLATFORM_H
#define PLATFORM_H

#define INCLUDE_FILEIO
#define INCLUDE_THREADING
#define INCLUDE_STRTOF

#ifndef NULL
#define NULL ((void *)0)
#endif

#define STDIN_FILENO   0
#define STDOUT_FILENO  1
#define STDERR_FILENO  2

// fcntl.h
#define O_RDONLY	00000000
#define O_WRONLY	00000001
#define O_RDWR		00000002
#define O_CREAT		00000100
#define O_APPEND        00002000
#define AT_FDCWD        -100

// mman.h
#define PROT_READ       0x1
#define PROT_WRITE      0x2
#define MAP_SHARED      0x001
#define MAP_PRIVATE     0x002
#define MAP_ANONYMOUS   0x020
#define MAP_GROWSDOWN   0x100
#define MAP_STACK       0x200

// futex.h
#define FUTEX_WAIT      0
#define FUTEX_WAKE      1

struct timespec
{
	unsigned long long  sec;
	unsigned long long  nsec;
};

struct stat
{
	unsigned long long  st_dev;
	unsigned long long  st_ino;
	unsigned long long  st_nlink;
	unsigned int        st_mode;
	unsigned int        st_uid;
	unsigned int        st_gid;
	unsigned int        __pad0;
	unsigned long long  st_rdev;
	unsigned long long  st_size;
	long long           st_blksize;
	long long           st_blocks;
	struct timespec     st_atim;
	struct timespec     st_mtim;
	struct timespec     st_ctim;
	long long           __unused[4]; // GCC requires add. 8 bytes padding
};

void                exit(int code);

int                 futex(unsigned int *uaddr, int futex_op, unsigned int val,
                          const struct timespec *timeout, unsigned int *uaddr2,
                          unsigned int val3);

#ifdef INCLUDE_FILEIO
int                 openat(int dirfd, const char *pathname, int flags);
int                 fstatat(int dirfd, const char *pathname,
                            struct stat *restrict statbuf, int flags);
int                 close(int fd);
#endif

long long           write(int fd, const void *buf, unsigned long long cnt);

void                *mmap(void *ptr, unsigned long long len, int prot,
                          int flags, int fd, unsigned long long ofs);
int                 munmap(void *ptr, unsigned long long len);

void                *malloc(unsigned long long len);
void                free(void *ptr);

void                *memset(void *dst, int c, unsigned long long len);
void                *memcpy(void *restrict dst, const void *restrict src,
                            unsigned long long len);

#ifdef INCLUDE_THREADING
void                *create_stack(unsigned long stacksz);
void                release_stack(void *stack, unsigned long stacksz);

long long           create_thread(void (*fn)(void *), void *param, void *stack);
#endif

unsigned long long  strlen(const char *s);
int                 strcmp(const char *s1, const char *s2);
int                 strncmp(const char *s1, const char *s2,
                            unsigned long long n);
char                *strstr(const char *str, const char *sub);

void                _putchar(char character); // For external custom printf

int                 isdigit(int c);
int                 isspace(int c);

int                 atoi(const char *s);
#ifdef INCLUDE_STRTOF
float               strtof(const char *restrict s, char **restrict endptr);
float               atof(const char *s);
#endif

#endif
