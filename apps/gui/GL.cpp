#include "GL.hpp"

#include <string>

namespace gl {

#define MESHREPAIR_GL_DEFINE(ret, name, args) PFN_##name name = nullptr;
MESHREPAIR_GL_FUNCTIONS(MESHREPAIR_GL_DEFINE)
#undef MESHREPAIR_GL_DEFINE

const char* load(GetProcAddress get_proc)
{
#define MESHREPAIR_GL_LOAD(ret, name, args)                                                                           \
    name = reinterpret_cast<PFN_##name>(get_proc("gl" #name));                                                        \
    if (!name)                                                                                                        \
        return "gl" #name;
    MESHREPAIR_GL_FUNCTIONS(MESHREPAIR_GL_LOAD)
#undef MESHREPAIR_GL_LOAD
    return nullptr;
}

} // namespace gl
