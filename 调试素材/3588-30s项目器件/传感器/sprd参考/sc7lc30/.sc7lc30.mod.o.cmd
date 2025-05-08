cmd_/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o := /home/stephen/work/t710/XR_T710/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-gcc -Wp,-MD,/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/.sc7lc30.mod.o.d  -nostdinc -isystem /home/stephen/work/t710/XR_T710/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/../lib/gcc/aarch64-linux-android/4.9.x/include -I/home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include -I./arch/arm64/include/generated  -I/home/stephen/work/t710/XR_T710/kernel4.14/include -I./include -I/home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi -I./arch/arm64/include/generated/uapi -I/home/stephen/work/t710/XR_T710/kernel4.14/include/uapi -I./include/generated/uapi -include /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kconfig.h  -I/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30 -I/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30 -D__KERNEL__ -mlittle-endian -Wall -Werror -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fshort-wchar -Werror-implicit-function-declaration -Wno-format-security -std=gnu89 -fno-PIE -mgeneral-regs-only -DCONFIG_AS_LSE=1 -fno-asynchronous-unwind-tables -fno-pic -mabi=lp64 -fno-delete-null-pointer-checks -Os -Wno-maybe-uninitialized --param=allow-store-data-races=0 -DCC_HAVE_ASM_GOTO -Wframe-larger-than=2048 -fstack-protector-strong -Wno-unused-but-set-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-var-tracking-assignments -g -pg -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-merge-all-constants -fmerge-constants -fno-stack-check -fconserve-stack -Werror=implicit-int -Werror=strict-prototypes -Werror=date-time  -DKBUILD_BASENAME='"sc7lc30.mod"'  -DKBUILD_MODNAME='"sc7lc30"' -DMODULE -mcmodel=large  -c -o /home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o /home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.c

source_/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o := /home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.c

deps_/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o := \
    $(wildcard include/config/module/unload.h) \
    $(wildcard include/config/retpoline.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/compiler_types.h \
    $(wildcard include/config/have/arch/compiler/h.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
    $(wildcard include/config/gcov/kernel.h) \
    $(wildcard include/config/arch/use/builtin/bswap.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/module.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/sysfs.h) \
    $(wildcard include/config/modules/tree/lookup.h) \
    $(wildcard include/config/livepatch.h) \
    $(wildcard include/config/cfi/clang.h) \
    $(wildcard include/config/unused/symbols.h) \
    $(wildcard include/config/module/sig.h) \
    $(wildcard include/config/generic/bug.h) \
    $(wildcard include/config/kallsyms.h) \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/tracepoints.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/event/tracing.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
    $(wildcard include/config/constructors.h) \
    $(wildcard include/config/strict/module/rwx.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/list.h \
    $(wildcard include/config/debug/list.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/types.h \
    $(wildcard include/config/have/uid16.h) \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/arch/dma/addr/t/64bit.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/types.h \
  arch/arm64/include/generated/uapi/asm/types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/int-ll64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/int-ll64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/bitsperlong.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitsperlong.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/bitsperlong.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/posix_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/stddef.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/stddef.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/posix_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/posix_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/poison.h \
    $(wildcard include/config/illegal/pointer/value.h) \
    $(wildcard include/config/page/poisoning/zero.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/const.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kernel.h \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/atomic/sleep.h) \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/arch/has/refcount.h) \
    $(wildcard include/config/panic/timeout.h) \
  /home/stephen/work/t710/XR_T710/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/lib/gcc/aarch64-linux-android/4.9.x/include/stdarg.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/linkage.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/stringify.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/export.h \
    $(wildcard include/config/have/underscore/symbol/prefix.h) \
    $(wildcard include/config/modversions.h) \
    $(wildcard include/config/module/rel/crcs.h) \
    $(wildcard include/config/trim/unused/ksyms.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/linkage.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/stack/validation.h) \
    $(wildcard include/config/kasan.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/barrier.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/barrier.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/bitops.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/bitops.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/builtin-__ffs.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/builtin-ffs.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/builtin-__fls.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/builtin-fls.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/ffz.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/fls64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/find.h \
    $(wildcard include/config/generic/find/first/bit.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/sched.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/hweight.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/arch_hweight.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/const_hweight.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/lock.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/non-atomic.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bitops/le.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/byteorder.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/byteorder/little_endian.h \
    $(wildcard include/config/cpu/big/endian.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/byteorder/little_endian.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/swab.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/swab.h \
  arch/arm64/include/generated/uapi/asm/swab.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/swab.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/byteorder/generic.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/log2.h \
    $(wildcard include/config/arch/has/ilog2/u32.h) \
    $(wildcard include/config/arch/has/ilog2/u64.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/typecheck.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/printk.h \
    $(wildcard include/config/message/loglevel/default.h) \
    $(wildcard include/config/console/loglevel/default.h) \
    $(wildcard include/config/early/printk.h) \
    $(wildcard include/config/printk/nmi.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/init.h \
    $(wildcard include/config/strict/kernel/rwx.h) \
    $(wildcard include/config/lto/clang.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kern_levels.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/cache.h \
    $(wildcard include/config/arch/has/cache/line/size.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/kernel.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/sysinfo.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/cache.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/cputype.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/sysreg.h \
    $(wildcard include/config/broken/gas/inst.h) \
    $(wildcard include/config/arm64/4k/pages.h) \
    $(wildcard include/config/arm64/16k/pages.h) \
    $(wildcard include/config/arm64/64k/pages.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/dynamic_debug.h \
    $(wildcard include/config/jump/label.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/jump_label.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/jump_label.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/insn.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/build_bug.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/stat.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/stat.h \
    $(wildcard include/config/compat.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/stat.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/stat.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/compat.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sched.h \
    $(wildcard include/config/sprd/core/ctl.h) \
    $(wildcard include/config/virt/cpu/accounting/native.h) \
    $(wildcard include/config/sched/info.h) \
    $(wildcard include/config/schedstats.h) \
    $(wildcard include/config/fair/group/sched.h) \
    $(wildcard include/config/sched/walt.h) \
    $(wildcard include/config/rt/group/sched.h) \
    $(wildcard include/config/thread/info/in/task.h) \
    $(wildcard include/config/cgroup/sched.h) \
    $(wildcard include/config/preempt/notifiers.h) \
    $(wildcard include/config/blk/dev/io/trace.h) \
    $(wildcard include/config/preempt/rcu.h) \
    $(wildcard include/config/tasks/rcu.h) \
    $(wildcard include/config/memcg.h) \
    $(wildcard include/config/slob.h) \
    $(wildcard include/config/compat/brk.h) \
    $(wildcard include/config/cgroups.h) \
    $(wildcard include/config/cc/stackprotector.h) \
    $(wildcard include/config/arch/has/scaled/cputime.h) \
    $(wildcard include/config/cpu/freq/times.h) \
    $(wildcard include/config/virt/cpu/accounting/gen.h) \
    $(wildcard include/config/no/hz/full.h) \
    $(wildcard include/config/posix/timers.h) \
    $(wildcard include/config/sysvipc.h) \
    $(wildcard include/config/detect/hung/task.h) \
    $(wildcard include/config/auditsyscall.h) \
    $(wildcard include/config/rt/mutexes.h) \
    $(wildcard include/config/debug/mutexes.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/lockdep.h) \
    $(wildcard include/config/lockdep/crossrelease.h) \
    $(wildcard include/config/ubsan.h) \
    $(wildcard include/config/block.h) \
    $(wildcard include/config/task/xacct.h) \
    $(wildcard include/config/cpusets.h) \
    $(wildcard include/config/intel/rdt.h) \
    $(wildcard include/config/futex.h) \
    $(wildcard include/config/perf/events.h) \
    $(wildcard include/config/debug/preempt.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/numa/balancing.h) \
    $(wildcard include/config/task/delay/acct.h) \
    $(wildcard include/config/fault/injection.h) \
    $(wildcard include/config/latencytop.h) \
    $(wildcard include/config/function/graph/tracer.h) \
    $(wildcard include/config/kcov.h) \
    $(wildcard include/config/uprobes.h) \
    $(wildcard include/config/bcache.h) \
    $(wildcard include/config/vmap/stack.h) \
    $(wildcard include/config/security.h) \
    $(wildcard include/config/preempt.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/sched.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/current.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/pid.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rculist.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rcupdate.h \
    $(wildcard include/config/preempt/count.h) \
    $(wildcard include/config/rcu/stall/common.h) \
    $(wildcard include/config/rcu/nocb/cpu.h) \
    $(wildcard include/config/tree/rcu.h) \
    $(wildcard include/config/tiny/rcu.h) \
    $(wildcard include/config/debug/objects/rcu/head.h) \
    $(wildcard include/config/hotplug/cpu.h) \
    $(wildcard include/config/prove/rcu.h) \
    $(wildcard include/config/debug/lock/alloc.h) \
    $(wildcard include/config/rcu/boost.h) \
    $(wildcard include/config/arch/weak/release/acquire.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/atomic.h \
    $(wildcard include/config/generic/atomic64.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/atomic.h \
    $(wildcard include/config/arm64/lse/atomics.h) \
    $(wildcard include/config/as/lse.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/lse.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/alternative.h \
    $(wildcard include/config/arm64/uao.h) \
    $(wildcard include/config/foo.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/cpucaps.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/atomic_lse.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/cmpxchg.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/bug.h \
    $(wildcard include/config/bug/on/data/corruption.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/bug.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/asm-bug.h \
    $(wildcard include/config/debug/bugverbose.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/brk-imm.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/bug.h \
    $(wildcard include/config/bug.h) \
    $(wildcard include/config/generic/bug/relative/pointers.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/atomic-long.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/irqflags.h \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/irqflags.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/ptrace.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/ptrace.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/hwcap.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/hwcap.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/ptrace.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/preempt.h \
  arch/arm64/include/generated/asm/preempt.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/preempt.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/thread_info.h \
    $(wildcard include/config/have/arch/within/stack/frames.h) \
    $(wildcard include/config/hardened/usercopy.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/restart_block.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/thread_info.h \
    $(wildcard include/config/arm64/sw/ttbr0/pan.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/memory.h \
    $(wildcard include/config/arm64/va/bits.h) \
    $(wildcard include/config/debug/align/rodata.h) \
    $(wildcard include/config/blk/dev/initrd.h) \
    $(wildcard include/config/debug/virtual.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/page-def.h \
    $(wildcard include/config/arm64/page/shift.h) \
    $(wildcard include/config/arm64/cont/shift.h) \
  arch/arm64/include/generated/asm/sizes.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/sizes.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sizes.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/mmdebug.h \
    $(wildcard include/config/debug/vm.h) \
    $(wildcard include/config/debug/vm/pgflags.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/discontigmem.h) \
    $(wildcard include/config/sparsemem.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/pfn.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/stack_pointer.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/bottom_half.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/lockdep.h \
    $(wildcard include/config/lock/stat.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/processor.h \
    $(wildcard include/config/have/hw/breakpoint.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/string.h \
    $(wildcard include/config/binary/printf.h) \
    $(wildcard include/config/fortify/source.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/string.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/string.h \
    $(wildcard include/config/arch/has/uaccess/flushcache.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/fpsimd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/hw_breakpoint.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/cpufeature.h \
    $(wildcard include/config/arm64/ssbd.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/virt.h \
    $(wildcard include/config/arm64/vhe.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/sections.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/sections.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/pgtable-hwdef.h \
    $(wildcard include/config/pgtable/levels.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/cpumask.h \
    $(wildcard include/config/cpumask/offstack.h) \
    $(wildcard include/config/debug/per/cpu/maps.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/threads.h \
    $(wildcard include/config/nr/cpus.h) \
    $(wildcard include/config/base/small.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/bitmap.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rcutree.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sem.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/time64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/time.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/math64.h \
    $(wildcard include/config/arch/supports/int128.h) \
  arch/arm64/include/generated/asm/div64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/div64.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/sem.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/ipc.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/spinlock.h \
    $(wildcard include/config/debug/spinlock.h) \
    $(wildcard include/config/generic/lockbreak.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/spinlock_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/spinlock_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rwlock_types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/spinlock.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rwlock.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/spinlock_api_smp.h \
    $(wildcard include/config/inline/spin/lock.h) \
    $(wildcard include/config/inline/spin/lock/bh.h) \
    $(wildcard include/config/inline/spin/lock/irq.h) \
    $(wildcard include/config/inline/spin/lock/irqsave.h) \
    $(wildcard include/config/inline/spin/trylock.h) \
    $(wildcard include/config/inline/spin/trylock/bh.h) \
    $(wildcard include/config/uninline/spin/unlock.h) \
    $(wildcard include/config/inline/spin/unlock/bh.h) \
    $(wildcard include/config/inline/spin/unlock/irq.h) \
    $(wildcard include/config/inline/spin/unlock/irqrestore.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rwlock_api_smp.h \
    $(wildcard include/config/inline/read/lock.h) \
    $(wildcard include/config/inline/write/lock.h) \
    $(wildcard include/config/inline/read/lock/bh.h) \
    $(wildcard include/config/inline/write/lock/bh.h) \
    $(wildcard include/config/inline/read/lock/irq.h) \
    $(wildcard include/config/inline/write/lock/irq.h) \
    $(wildcard include/config/inline/read/lock/irqsave.h) \
    $(wildcard include/config/inline/write/lock/irqsave.h) \
    $(wildcard include/config/inline/read/trylock.h) \
    $(wildcard include/config/inline/write/trylock.h) \
    $(wildcard include/config/inline/read/unlock.h) \
    $(wildcard include/config/inline/write/unlock.h) \
    $(wildcard include/config/inline/read/unlock/bh.h) \
    $(wildcard include/config/inline/write/unlock/bh.h) \
    $(wildcard include/config/inline/read/unlock/irq.h) \
    $(wildcard include/config/inline/write/unlock/irq.h) \
    $(wildcard include/config/inline/read/unlock/irqrestore.h) \
    $(wildcard include/config/inline/write/unlock/irqrestore.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/uidgid.h \
    $(wildcard include/config/multiuser.h) \
    $(wildcard include/config/user/ns.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/highuid.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rhashtable.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/err.h \
  arch/arm64/include/generated/uapi/asm/errno.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/errno.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/errno-base.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/errno.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/errno.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/jhash.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/unaligned/packed_struct.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/list_nulls.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/workqueue.h \
    $(wildcard include/config/debug/objects/work.h) \
    $(wildcard include/config/freezer.h) \
    $(wildcard include/config/wq/watchdog.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/timer.h \
    $(wildcard include/config/debug/objects/timers.h) \
    $(wildcard include/config/no/hz/common.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/ktime.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/time.h \
    $(wildcard include/config/arch/uses/gettimeoffset.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/seqlock.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/jiffies.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/timex.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/timex.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/param.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/param.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/param.h \
    $(wildcard include/config/hz.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/param.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/timex.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/arch_timer.h \
    $(wildcard include/config/arm/arch/timer/ool/workaround.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/smp.h \
    $(wildcard include/config/up/late/init.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/llist.h \
    $(wildcard include/config/arch/have/nmi/safe/cmpxchg.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/smp.h \
    $(wildcard include/config/arm64/acpi/parking/protocol.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/percpu.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/percpu.h \
    $(wildcard include/config/have/setup/per/cpu/area.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/percpu-defs.h \
    $(wildcard include/config/debug/force/weak/per/cpu.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/clocksource/arm_arch_timer.h \
    $(wildcard include/config/arm/arch/timer.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/timecounter.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/timex.h \
  include/generated/timeconst.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/timekeeping.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/debugobjects.h \
    $(wildcard include/config/debug/objects.h) \
    $(wildcard include/config/debug/objects/free.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/mutex.h \
    $(wildcard include/config/mutex/spin/on/owner.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/osq_lock.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/debug_locks.h \
    $(wildcard include/config/debug/locking/api/selftests.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/ipc.h \
  arch/arm64/include/generated/uapi/asm/ipcbuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/ipcbuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/refcount.h \
    $(wildcard include/config/refcount/full.h) \
  arch/arm64/include/generated/uapi/asm/sembuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/sembuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/shm.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/page.h \
    $(wildcard include/config/have/arch/pfn/valid.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/personality.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/personality.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/pgtable-types.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/pgtable-nopud.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/pgtable-nop4d-hack.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/5level-fixup.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/getorder.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/shm.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/hugetlb_encode.h \
  arch/arm64/include/generated/uapi/asm/shmbuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/shmbuf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/shmparam.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/shmparam.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kcov.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/kcov.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/plist.h \
    $(wildcard include/config/debug/pi/list.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/hrtimer.h \
    $(wildcard include/config/high/res/timers.h) \
    $(wildcard include/config/time/low/res.h) \
    $(wildcard include/config/timerfd.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rbtree.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/percpu.h \
    $(wildcard include/config/need/per/cpu/embed/first/chunk.h) \
    $(wildcard include/config/need/per/cpu/page/first/chunk.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/timerqueue.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/seccomp.h \
    $(wildcard include/config/seccomp.h) \
    $(wildcard include/config/have/arch/seccomp/filter.h) \
    $(wildcard include/config/seccomp/filter.h) \
    $(wildcard include/config/checkpoint/restore.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/seccomp.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/seccomp.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/unistd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/unistd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/unistd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/unistd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/seccomp.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/unistd.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/nodemask.h \
    $(wildcard include/config/highmem.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/numa.h \
    $(wildcard include/config/nodes/shift.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/resource.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/resource.h \
  arch/arm64/include/generated/uapi/asm/resource.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/resource.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/resource.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/latencytop.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sched/prio.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/signal_types.h \
    $(wildcard include/config/old/sigaction.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/signal.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/signal.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/signal.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/signal.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/signal-defs.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/sigcontext.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/uapi/asm/siginfo.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/asm-generic/siginfo.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/mm_types_task.h \
    $(wildcard include/config/arch/want/batched/unmap/tlb/flush.h) \
    $(wildcard include/config/split/ptlock/cpus.h) \
    $(wildcard include/config/arch/enable/split/pmd/ptlock.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/task_io_accounting.h \
    $(wildcard include/config/task/io/accounting.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sched/task_stack.h \
    $(wildcard include/config/stack/growsup.h) \
    $(wildcard include/config/debug/stack/usage.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/magic.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/stat.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kmod.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/umh.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/gfp.h \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/zone/dma32.h) \
    $(wildcard include/config/zone/device.h) \
    $(wildcard include/config/pm/sleep.h) \
    $(wildcard include/config/memory/isolation.h) \
    $(wildcard include/config/compaction.h) \
    $(wildcard include/config/cma.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/mmzone.h \
    $(wildcard include/config/force/max/zoneorder.h) \
    $(wildcard include/config/zsmalloc.h) \
    $(wildcard include/config/memory/hotplug.h) \
    $(wildcard include/config/flat/node/mem/map.h) \
    $(wildcard include/config/page/extension.h) \
    $(wildcard include/config/no/bootmem.h) \
    $(wildcard include/config/deferred/struct/page/init.h) \
    $(wildcard include/config/transparent/hugepage.h) \
    $(wildcard include/config/have/memory/present.h) \
    $(wildcard include/config/have/memoryless/nodes.h) \
    $(wildcard include/config/need/node/memmap/size.h) \
    $(wildcard include/config/have/memblock/node/map.h) \
    $(wildcard include/config/need/multiple/nodes.h) \
    $(wildcard include/config/have/arch/early/pfn/to/nid.h) \
    $(wildcard include/config/sparsemem/extreme.h) \
    $(wildcard include/config/memory/hotremove.h) \
    $(wildcard include/config/holes/in/zone.h) \
    $(wildcard include/config/arch/has/holes/memorymodel.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/wait.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/wait.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/pageblock-flags.h \
    $(wildcard include/config/hugetlb/page.h) \
    $(wildcard include/config/hugetlb/page/size/variable.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/page-flags-layout.h \
  include/generated/bounds.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/sparsemem.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/memory_hotplug.h \
    $(wildcard include/config/arch/has/add/pages.h) \
    $(wildcard include/config/have/arch/nodedata/extension.h) \
    $(wildcard include/config/have/bootmem/info/node.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/notifier.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rwsem.h \
    $(wildcard include/config/rwsem/spin/on/owner.h) \
    $(wildcard include/config/rwsem/generic/spinlock.h) \
  arch/arm64/include/generated/asm/rwsem.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/rwsem.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/srcu.h \
    $(wildcard include/config/tiny/srcu.h) \
    $(wildcard include/config/tree/srcu.h) \
    $(wildcard include/config/srcu.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rcu_segcblist.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/srcutree.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rcu_node_tree.h \
    $(wildcard include/config/rcu/fanout.h) \
    $(wildcard include/config/rcu/fanout/leaf.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/completion.h \
    $(wildcard include/config/lockdep/completions.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/topology.h \
    $(wildcard include/config/use/percpu/numa/node/id.h) \
    $(wildcard include/config/sched/smt.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/topology.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/arch_topology.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/topology.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sysctl.h \
    $(wildcard include/config/sysctl.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/sysctl.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/elf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/elf.h \
  arch/arm64/include/generated/asm/user.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/user.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/elf.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/uapi/linux/elf-em.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kobject.h \
    $(wildcard include/config/uevent/helper.h) \
    $(wildcard include/config/debug/kobject/release.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/sysfs.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kernfs.h \
    $(wildcard include/config/kernfs.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/idr.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/radix-tree.h \
    $(wildcard include/config/radix/tree/multiorder.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kobject_ns.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/kref.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/moduleparam.h \
    $(wildcard include/config/alpha.h) \
    $(wildcard include/config/ia64.h) \
    $(wildcard include/config/ppc64.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/rbtree_latch.h \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/cfi.h \
    $(wildcard include/config/cfi/clang/shadow.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/arch/arm64/include/asm/module.h \
    $(wildcard include/config/arm64/module/plts.h) \
    $(wildcard include/config/dynamic/ftrace.h) \
    $(wildcard include/config/randomize/base.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/asm-generic/module.h \
    $(wildcard include/config/have/mod/arch/specific.h) \
    $(wildcard include/config/modules/use/elf/rel.h) \
    $(wildcard include/config/modules/use/elf/rela.h) \
  /home/stephen/work/t710/XR_T710/kernel4.14/include/linux/vermagic.h \
  include/generated/utsrelease.h \

/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o: $(deps_/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o)

$(deps_/home/stephen/work/t710/XR_T710/vendor/sprd/modules/devdrv/input/misc/sc7lc30/sc7lc30.mod.o):
