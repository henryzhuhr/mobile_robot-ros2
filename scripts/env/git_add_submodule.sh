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

rm -rf core
git rm -r --cached core
git submodule add -b dev git@github.com:HenryZhuHR/mobile_robot-core.git  core
