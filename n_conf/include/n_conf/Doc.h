// Copyright by BeeX [2026]

#ifndef N_CONF_DOC_H
#define N_CONF_DOC_H

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace conf {

class Doc {
public:
    bool load(const std::string &path,
              const std::vector<std::string> &own,
              const std::vector<std::string> &borrow = std::vector<std::string>());

    double      num(const std::string &key);
    int         integer(const std::string &key);
    bool        flag(const std::string &key);
    std::string text(const std::string &key);

    void nums(const std::string &key, double *out, size_t count);

    static const char *const JOINT[4];
    void perJoint(const std::string &key, double *out);

    bool ok() const { return problems_.empty() && unused().empty(); }
    const std::vector<std::string> &problems() const { return problems_; }

    std::vector<std::string> unused() const;
    std::string report() const;
    std::string effective() const;

private:
    const std::string *find(const std::string &key);

    bool owned(const std::string &key) const;

    std::map<std::string, std::string> leaf_;   // dotted key -> scalar text
    std::map<std::string, std::vector<std::string>> list_;
    std::set<std::string>              read_;
    std::set<std::string>              own_;
    std::vector<std::string>           problems_;
};

}  // namespace conf

#endif  // N_CONF_DOC_H
