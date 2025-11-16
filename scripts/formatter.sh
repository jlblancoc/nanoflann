# formatter.sh
find include/ examples/ tests/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | grep -v gtest | xargs clang-format-14 -i
