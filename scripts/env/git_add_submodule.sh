# usage: git submodule [--quiet] [--cached]
#    or: git submodule [--quiet] add [-b <branch>] [-f|--force] [--name <name>] [--reference <repository>] [--] <repository> [<path>]
#    or: git submodule [--quiet] status [--cached] [--recursive] [--] [<path>...]
#    or: git submodule [--quiet] init [--] [<path>...]
#    or: git submodule [--quiet] deinit [-f|--force] (--all| [--] <path>...)
#    or: git submodule [--quiet] update [--init [--filter=<filter-spec>]] [--remote] [-N|--no-fetch] [-f|--force] [--checkout|--merge|--rebase] [--[no-]recommend-shallow] [--reference <repository>] [--recursive] [--[no-]single-branch] [--] [<path>...]
#    or: git submodule [--quiet] set-branch (--default|--branch <branch>) [--] <path>
#    or: git submodule [--quiet] set-url [--] <path> <newurl>
#    or: git submodule [--quiet] summary [--cached|--files] [--summary-limit <n>] [commit] [--] [<path>...]
#    or: git submodule [--quiet] foreach [--recursive] <command>
#    or: git submodule [--quiet] sync [--recursive] [--] [<path>...]
#    or: git submodule [--quiet] absorbgitdirs [--] [<path>...]

rm -rf modules/controller
git rm -r --cached modules/controller
git submodule add -b main -f git@github.com:HenryZhuHR/mobile_robot-controller.git  modules/controller

rm -rf modules/manager
git rm -r --cached modules/manager
git submodule add -b main git@github.com:HenryZhuHR/mobile_robot-manager.git  modules/manager

# cd modules/controller
rm -rf motion_manager/seiral
git rm -r --cached motion_manager/seiral
git submodule add -b main git@github.com:HenryZhuHR/serial-ros2.git motion_manager/serial
