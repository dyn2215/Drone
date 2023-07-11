file(REMOVE_RECURSE
  "../etc/init.d/rc.autostart"
  "../etc/init.d/rc.autostart.post"
  "../etc/init.d/rc.serial"
  "../romfs_files.tar"
  "CMakeFiles/romfs_gen_files_target"
  "romfs_copy.stamp"
  "romfs_extract.stamp"
  "romfs_extras.stamp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/romfs_gen_files_target.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
