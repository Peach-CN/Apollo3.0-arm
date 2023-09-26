// Copyright 2015 Baidu Inc. All Rights Reserved.
// @author: Ziqiang Huan (huanziqiang@baidu.com)
// @author: Yangguang Li (liyangguang@baidu.com)
// @file: file_util.h
// @brief:
#ifndef ADU_PERCEPTION_LIB_BASE_FILE_UTIL_H
#define ADU_PERCEPTION_LIB_BASE_FILE_UTIL_H

#include <string>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "modules/perception/lib/base/noncopyable.h"

namespace apollo {
namespace perception {

// 常用的文件操作工具类
enum FileType {
    TYPE_FILE, TYPE_DIR
};

// file name compared type
enum FileCompareType {
    FCT_DIGITAL = 0,
    FCT_LEXICOGRAPHICAL = 1,
    FCT_UNKNOWN = 8
};

class FileUtil {
public:
    FileUtil() { }

    ~FileUtil() { }

    // 判断文件或目录是否存在
    static bool exists(const std::string &filename);

    // check whether file exists with [suffix] extension in [path]
    static bool exists(const std::string& path, const std::string& suffix);

    // 判断是文件类型: 目录或文件
    static bool get_type(const std::string &filename, FileType *type);

    // 删除文件,包括目录和文件
    static bool delete_file(const std::string &filename);

    // 重命名文件
    static bool rename_file(
        const std::string &old_file,
        const std::string &new_file);

    // 创建文件夹
    static bool create_dir(const std::string &dir);

    static bool get_file_content(const std::string& path, std::string *content);
    static bool read_lines(const std::string& path, std::vector<std::string>* lines);

    static std::string remove_file_suffix(std::string filename);

    // TODO(@huanziqiang): this function stay just for compatibility,
    //                     should be removed later
    static void get_file_list(
            std::vector<std::string>& files,
            const std::string path,
            const std::string suffix = "");

    static void get_file_list(const std::string& path,
                              const std::string& suffix,
                              std::vector<std::string>* files);

    static void get_file_list(const std::string& path,
                              std::vector<std::string>* files);

    static std::string get_absolute_path(
            const std::string& prefix,
            const std::string& relative_path);

    // get file name
    // "/home/work/data/1.txt" -> 1
    static void get_file_name(
            const std::string& file,
            std::string* name);

    // return -1 when error occurred
    static int num_lines(const std::string& filename);

    // compare two file's name by digital value
    // "/home/work/data/1.txt" < "/home/user/data/10.txt"
    // "1.txt" < "./data/2.txt"
    static bool compare_file_by_digital(
            const std::string& file_left,
            const std::string& file_right);

    // compare two file's name by lexicographical order
    static bool compare_file_by_lexicographical(
            const std::string& file_left,
            const std::string& file_right);

private:
    static bool compare_file(
            const std::string& file_left,
            const std::string& file_right,
            FileCompareType type);

    DISALLOW_COPY_AND_ASSIGN(FileUtil);
};

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_LIB_BASE_FILE_UTIL_H
