/*
 * build_info.h
 *
 * Which build is running: "<git hash>[-dirty] <date> <time>", e.g.
 * "46ac052-dirty Nov 15 2026 16:02:11". The hash and dirty flag come from
 * src/build_info_gen.h, which firmware/makefile.init writes on every build
 * (git-ignored); "-dirty" means the firmware directory had uncommitted
 * changes when it was built. Printed in the boot banner and stored with
 * every black-box recording.
 */

#ifndef BUILD_INFO_H_
#define BUILD_INFO_H_

extern char const g_build_id[];

#endif /* BUILD_INFO_H_ */
