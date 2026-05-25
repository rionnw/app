#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include <min2phase/min2phase.hpp>

namespace {

char* copy_string(const std::string& value) {
    auto* out = static_cast<char*>(std::malloc(value.size() + 1));
    if (out == nullptr) {
        return nullptr;
    }
    std::memcpy(out, value.c_str(), value.size() + 1);
    return out;
}

} // namespace

extern "C" int robo_min2phase_solve(const char* facelets, char** solution, char** error) {
    if (solution == nullptr || error == nullptr) {
        return -100;
    }
    *solution = nullptr;
    *error = nullptr;

    if (facelets == nullptr) {
        *error = copy_string("facelets pointer is null");
        return -101;
    }

    try {
        min2phase::SolveOptions options;
        const auto result = min2phase::solve(facelets, options);
        if (!result.ok()) {
            *error = copy_string(result.errorMessage());
            return static_cast<int>(result.status);
        }

        *solution = copy_string(result.solution);
        return 0;
    } catch (const std::exception& ex) {
        *error = copy_string(ex.what());
        return -102;
    } catch (...) {
        *error = copy_string("unknown min2phase failure");
        return -103;
    }
}

extern "C" void robo_min2phase_free(char* value) {
    std::free(value);
}
