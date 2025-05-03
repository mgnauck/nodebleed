#include "platform.h"

#define SYS_CLONE      220

#define CLONE_VM       0x00000100 // Both processes share memory space
#define CLONE_FS       0x00000200 // Share filesystem information
#define CLONE_FILES    0x00000400 // Share file descriptor table
#define CLONE_SIGHAND  0x00000800 // Shared table of signal handlers
#define CLONE_PARENT   0x00008000 // Same parent as calling process
#define CLONE_THREAD   0x00010000 // Share threadgroup of calling process
#define CLONE_IO       0x80000000 // Share I/O context

#ifdef __aarch64__

long long syscall(long long nr, long long a, long long b, long long c,
                  long long d, long long e, long long f)
{
	register long long x8 __asm("x8") = nr;
	register long long x0 __asm("x0") = a;
	register long long x1 __asm("x1") = b;
	register long long x2 __asm("x2") = c;
	register long long x3 __asm("x3") = d;
	register long long x4 __asm("x4") = e;
	register long long x5 __asm("x5") = f;
	__asm volatile (
		"svc #0"
		: "=r" (x0)
		: "r"(x8), "r"(x0), "r"(x1), "r"(x2), "r"(x3), "r"(x4), "r"(x5)
		: "memory"
	);
	return x0;
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

	register unsigned long long x8 __asm("x8") = SYS_CLONE; // Nr + ret val
	register unsigned long long x0 __asm("x0") = flags;
	register unsigned long long x1 __asm("x1") =
	  (unsigned long long)&top[-2];

	__asm volatile (
		"svc #0                 \n\t" // Child process returns with 0
		"cbnz x0, 0f            \n\t" // Parent branches
		"ldp x0, x1, [sp], #16  \n\t" // Entry fn to x1, param to x0
		"ret x1                 \n\t" // "Call" entry func
		"0:                     \n\t" // Parent exits here
		: "=r"(x8)
		: "r"(x8), "r"(x0), "r"(x1)
		: "memory", "cc"
	);

	return x8;
}
#endif

#endif
