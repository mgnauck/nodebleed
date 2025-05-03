#include "platform.h"

#define SYS_CLONE      56

#define CLONE_VM       0x00000100 // Both processes share memory space
#define CLONE_FS       0x00000200 // Share filesystem information
#define CLONE_FILES    0x00000400 // Share file descriptor table
#define CLONE_SIGHAND  0x00000800 // Shared table of signal handlers
#define CLONE_PARENT   0x00008000 // Same parent as calling process
#define CLONE_THREAD   0x00010000 // Share threadgroup of calling process
#define CLONE_IO       0x80000000 // Share I/O context

#ifdef __x86_64__

long long syscall(long long nr, long long a, long long b, long long c,
                  long long d, long long e, long long f)
{
	long long res;
	register long long r10 __asm("r10") = d;
	register long long r8 __asm("r8") = e;
	register long long r9 __asm("r9") = f;
	__asm volatile (
		"syscall"
		: "=a" (res)
		: "a"(nr), "D"(a), "S"(b), "d"(c), "r"(r10), "r"(r8), "r"(r9) 
		: "rcx", "r11", "cc", "memory"
	);
	return res;
}

#ifdef INCLUDE_THREADING
long long create_thread(void (*fn)(void *), void *param, void *stack)
{
	void **top = (void **)stack;
	// Avoid 'wrong' gcc warning via void ** cast
	top[-1] = *(void **)&fn; // Push thread entry function first
	top[-2] = param; // Push thread entry function parameter location next

	unsigned long long flags = CLONE_FS | CLONE_FILES | CLONE_SIGHAND |
	  CLONE_PARENT | CLONE_THREAD | CLONE_IO | CLONE_VM;

	long long res = 0;
	__asm volatile (
		"syscall            \n\t" // Child process returns with 0
		"andl %%eax, %%eax  \n\t" // Child sets zero flag
		"jnz 0f             \n\t" // Parent jumps
		"popq %%rdi         \n\t" // Entry func arg (2nd on stack)
		"ret                \n\t" // "Call" entry func (1st on stack)
		"0:                 \n\t" // Parent exits here
		: "=a"(res)
		: "0"(SYS_CLONE), "D"(flags), "S"(&top[-2])
		: "memory", "rcx", "r11", "cc"
	);

	return res;
}
#endif

#endif
