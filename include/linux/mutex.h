/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mutexes: blocking mutual exclusion locks
 *
 * started by Ingo Molnar:
 *
 *  Copyright (C) 2004, 2005, 2006 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 * This file contains the main data structure and API definitions.
 */
#ifndef __LINUX_MUTEX_H
#define __LINUX_MUTEX_H

#include <asm/current.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>
#include <linux/lockdep.h>
#include <linux/atomic.h>
#include <asm/processor.h>
#include <linux/osq_lock.h>
#include <linux/debug_locks.h>

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define __DEP_MAP_MUTEX_INITIALIZER(lockname)			\
		, .dep_map = {					\
			.name = #lockname,			\
			.wait_type_inner = LD_WAIT_SLEEP,	\
		}
#else
# define __DEP_MAP_MUTEX_INITIALIZER(lockname)
#endif

#ifndef CONFIG_PREEMPT_RT

/*
 * Simple, straightforward mutexes with strict semantics:
 *
 * - only one task can hold the mutex at a time
 * - only the owner can unlock the mutex
 * - multiple unlocks are not permitted
 * - recursive locking is not permitted
 * - a mutex object must be initialized via the API
 * - a mutex object must not be initialized via memset or copying
 * - task may not exit with mutex held
 * - memory areas where held locks reside must not be freed
 * - held mutexes must not be reinitialized
 * - mutexes may not be used in hardware or software interrupt
 *   contexts such as tasklets and timers
 *
 * These semantics are fully enforced when DEBUG_MUTEXES is
 * enabled. Furthermore, besides enforcing the above rules, the mutex
 * debugging code also implements a number of additional features
 * that make lock debugging easier and faster:
 *
 * - uses symbolic names of mutexes, whenever they are printed in debug output
 * - point-of-acquire tracking, symbolic lookup of function names
 * - list of all locks held in the system, printout of them
 * - owner tracking
 * - detects self-recursing locks and prints out all relevant info
 * - detects multi-task circular deadlocks and prints out all affected
 *   locks and tasks (and only those tasks)
 */
struct mutex {
	/**
	 * count成员是mutex锁的本体,为0则说明锁没有被持有,非空则说明锁被持有.
	 * count被分为两个区域: 3~63位存放锁的持有着的task_struct地址;
	 *                    1~2位实现handoff机制;
	 *                    0位表示wait_list是否为空;
	 * 
	 */
	atomic_long_t		owner; // 记录了锁的持有者的task_struct地址,且低3bit记录了锁的状态;
	raw_spinlock_t		wait_lock; // 用来保护wait_list链表的自旋锁
#ifdef CONFIG_MUTEX_SPIN_ON_OWNER
    /*
	*在定义了CONFIG_MUTEX_SPIN_ON_OWNER宏的时候,mutex支持乐观自选机制,osq成员就是乐观自旋需要持有的MCS队列.
	*optimistic_spin_queue只有一个tail成员,如果等于0,则表示一个空队列,没有乐观自旋的任务.
	* 则tail指向一个节点是optimistic_spin_queue对象队列的尾部(tail并不是一个指针,它是一个cpu number,optimistic_spin_queue对象是per-cpu的,有了cpu number就可以定位到其optimistic_spin_queue对象)
	*/
	struct optimistic_spin_queue osq; /* Spinner MCS lock */   
#endif
	struct list_head	wait_list; // 等待链表,当无法获取锁并且不具备乐观自旋条件的时候需要挂入这个等待队列等着拿锁的进程会被记录在此list上,操作wait_list需要wait_lock的保护
#ifdef CONFIG_DEBUG_MUTEXES
	void			*magic;
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
};

#ifdef CONFIG_DEBUG_MUTEXES

#define __DEBUG_MUTEX_INITIALIZER(lockname)				\
	, .magic = &lockname

extern void mutex_destroy(struct mutex *lock);

#else

# define __DEBUG_MUTEX_INITIALIZER(lockname)

static inline void mutex_destroy(struct mutex *lock) {}

#endif

/**
 * mutex_init - initialize the mutex
 * @mutex: the mutex to be initialized
 *
 * Initialize the mutex to unlocked state.
 *
 * It is not allowed to initialize an already locked mutex.
 */
#define mutex_init(mutex)						\
do {									\
	static struct lock_class_key __key;				\
									\
	__mutex_init((mutex), #mutex, &__key);				\
} while (0)

#define __MUTEX_INITIALIZER(lockname) \
		{ .owner = ATOMIC_LONG_INIT(0) \
		, .wait_lock = __RAW_SPIN_LOCK_UNLOCKED(lockname.wait_lock) \
		, .wait_list = LIST_HEAD_INIT(lockname.wait_list) \
		__DEBUG_MUTEX_INITIALIZER(lockname) \
		__DEP_MAP_MUTEX_INITIALIZER(lockname) }

#define DEFINE_MUTEX(mutexname) \
	struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

extern void __mutex_init(struct mutex *lock, const char *name,
			 struct lock_class_key *key);

/**
 * mutex_is_locked - is the mutex locked
 * @lock: the mutex to be queried
 *
 * Returns true if the mutex is locked, false if unlocked.
 */
extern bool mutex_is_locked(struct mutex *lock);

#else /* !CONFIG_PREEMPT_RT */
/*
 * Preempt-RT variant based on rtmutexes.
 */
#include <linux/rtmutex.h>

struct mutex {
	struct rt_mutex_base	rtmutex;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
};

#define __MUTEX_INITIALIZER(mutexname)					\
{									\
	.rtmutex = __RT_MUTEX_BASE_INITIALIZER(mutexname.rtmutex)	\
	__DEP_MAP_MUTEX_INITIALIZER(mutexname)				\
}

#define DEFINE_MUTEX(mutexname)						\
	struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

extern void __mutex_rt_init(struct mutex *lock, const char *name,
			    struct lock_class_key *key);
extern int mutex_trylock(struct mutex *lock);

static inline void mutex_destroy(struct mutex *lock) { }

#define mutex_is_locked(l)	rt_mutex_base_is_locked(&(l)->rtmutex)

#define __mutex_init(mutex, name, key)			\
do {							\
	rt_mutex_base_init(&(mutex)->rtmutex);		\
	__mutex_rt_init((mutex), name, key);		\
} while (0)

#define mutex_init(mutex)				\
do {							\
	static struct lock_class_key __key;		\
							\
	__mutex_init((mutex), #mutex, &__key);		\
} while (0)
#endif /* CONFIG_PREEMPT_RT */

/*
 * See kernel/locking/mutex.c for detailed documentation of these APIs.
 * Also see Documentation/locking/mutex-design.rst.
 */
#ifdef CONFIG_DEBUG_LOCK_ALLOC
extern void mutex_lock_nested(struct mutex *lock, unsigned int subclass);
extern void _mutex_lock_nest_lock(struct mutex *lock, struct lockdep_map *nest_lock);

extern int __must_check mutex_lock_interruptible_nested(struct mutex *lock,
					unsigned int subclass);
extern int __must_check mutex_lock_killable_nested(struct mutex *lock,
					unsigned int subclass);
extern void mutex_lock_io_nested(struct mutex *lock, unsigned int subclass);

#define mutex_lock(lock) mutex_lock_nested(lock, 0)
#define mutex_lock_interruptible(lock) mutex_lock_interruptible_nested(lock, 0)
#define mutex_lock_killable(lock) mutex_lock_killable_nested(lock, 0)
#define mutex_lock_io(lock) mutex_lock_io_nested(lock, 0)

#define mutex_lock_nest_lock(lock, nest_lock)				\
do {									\
	typecheck(struct lockdep_map *, &(nest_lock)->dep_map);	\
	_mutex_lock_nest_lock(lock, &(nest_lock)->dep_map);		\
} while (0)

#else
extern void mutex_lock(struct mutex *lock);
extern int __must_check mutex_lock_interruptible(struct mutex *lock);
extern int __must_check mutex_lock_killable(struct mutex *lock);
extern void mutex_lock_io(struct mutex *lock);

# define mutex_lock_nested(lock, subclass) mutex_lock(lock)
# define mutex_lock_interruptible_nested(lock, subclass) mutex_lock_interruptible(lock)
# define mutex_lock_killable_nested(lock, subclass) mutex_lock_killable(lock)
# define mutex_lock_nest_lock(lock, nest_lock) mutex_lock(lock)
# define mutex_lock_io_nested(lock, subclass) mutex_lock_io(lock)
#endif

/*
 * NOTE: mutex_trylock() follows the spin_trylock() convention,
 *       not the down_trylock() convention!
 *
 * Returns 1 if the mutex has been acquired successfully, and 0 on contention.
 */
extern int mutex_trylock(struct mutex *lock);
extern void mutex_unlock(struct mutex *lock);

extern int atomic_dec_and_mutex_lock(atomic_t *cnt, struct mutex *lock);

#endif /* __LINUX_MUTEX_H */
