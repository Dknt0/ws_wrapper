#ifndef JS_TEMP_HPP
#define JS_TEMP_HPP

#include <algorithm>
#include <cstddef>
#include <string>

template <size_t N>
struct CTString {
  constexpr CTString(const char (&str)[N]) { std::copy_n(str, N, value); }
  constexpr size_t size() const { return N; }
  char value[N]{};
};

template <CTString Str>
constexpr auto CTStringCat() {
  return Str;
}

template <CTString Str1, CTString Str2, CTString... Ctrs>
constexpr auto CTStringCat() {
  auto tail = CTStringCat<Str2, Ctrs...>();
  char front[Str1.size() + tail.size() - 1] = {};
  std::copy_n(Str1.value, Str1.size() - 1, front);
  std::copy_n(tail.value, tail.size(), front + Str1.size() - 1);
  CTString res(front);
  return res;
}

template <CTString Name, CTString Value>
constexpr auto ButtonMsg() {
  constexpr CTString str_1(
      "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"");
  constexpr CTString str_2("\\\",\\\"value\\\":");
  constexpr CTString str_3("}]}\"}");
  constexpr auto res = CTStringCat<str_1, Name, str_2, Value, str_3>();
  return res;
}

inline const std::string VelocityMsg(const std::string& name, double value) {
  int64_t vel_value = *reinterpret_cast<int64_t*>(&value);
  std::string msg = "{\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"" + name +
                    "\\\",\\\"value\\\":" + std::to_string(vel_value) +
                    "}]}\",\"type\":\"events\"}";
  return msg;
}
#endif  // JS_TEMP_HPP
