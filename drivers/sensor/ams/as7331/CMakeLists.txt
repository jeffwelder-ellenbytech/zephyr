# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(as7331.c)
zephyr_library_sources_ifdef(CONFIG_AS7331_TRIGGER as7331_trigger.c)
