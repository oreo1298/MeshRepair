// Minimal OpenGL 3.3 core function loader for the viewer.
//
// Only the handful of functions used by the viewer are declared, so that the
// GUI builds without any GL headers or loader libraries; the pointers are
// resolved at run time through GLFW (works with GLX and EGL, X11 and Wayland).

#pragma once

#include <cstddef>
#include <cstdint>

namespace gl {

using GLenum     = unsigned int;
using GLboolean  = unsigned char;
using GLbitfield = unsigned int;
using GLint      = int;
using GLuint     = unsigned int;
using GLsizei    = int;
using GLfloat    = float;
using GLdouble   = double;
using GLchar     = char;
using GLubyte    = unsigned char;
using GLsizeiptr = std::ptrdiff_t;
using GLintptr   = std::ptrdiff_t;

constexpr GLenum FALSE_                = 0;
constexpr GLenum TRUE_                 = 1;
constexpr GLenum DEPTH_BUFFER_BIT      = 0x00000100;
constexpr GLenum COLOR_BUFFER_BIT      = 0x00004000;
constexpr GLenum POINTS                = 0x0000;
constexpr GLenum LINES                 = 0x0001;
constexpr GLenum TRIANGLES             = 0x0004;
constexpr GLenum LEQUAL                = 0x0203;
constexpr GLenum LESS                  = 0x0201;
constexpr GLenum ALWAYS                = 0x0207;
constexpr GLenum SRC_ALPHA             = 0x0302;
constexpr GLenum ONE_MINUS_SRC_ALPHA   = 0x0303;
constexpr GLenum CULL_FACE             = 0x0B44;
constexpr GLenum DEPTH_TEST            = 0x0B71;
constexpr GLenum BLEND                 = 0x0BE2;
constexpr GLenum PACK_ALIGNMENT        = 0x0D05;
constexpr GLenum UNSIGNED_BYTE         = 0x1401;
constexpr GLenum UNSIGNED_INT          = 0x1405;
constexpr GLenum FLOAT                 = 0x1406;
constexpr GLenum RGB                   = 0x1907;
constexpr GLenum RGBA                  = 0x1908;
constexpr GLenum POLYGON_OFFSET_FILL   = 0x8037;
constexpr GLenum MULTISAMPLE           = 0x809D;
constexpr GLenum ARRAY_BUFFER          = 0x8892;
constexpr GLenum ELEMENT_ARRAY_BUFFER  = 0x8893;
constexpr GLenum STATIC_DRAW           = 0x88E4;
constexpr GLenum DYNAMIC_DRAW          = 0x88E8;
constexpr GLenum FRAGMENT_SHADER       = 0x8B30;
constexpr GLenum VERTEX_SHADER         = 0x8B31;
constexpr GLenum COMPILE_STATUS        = 0x8B81;
constexpr GLenum LINK_STATUS           = 0x8B82;
constexpr GLenum INFO_LOG_LENGTH       = 0x8B84;
constexpr GLenum PROGRAM_POINT_SIZE    = 0x8642;
constexpr GLenum READ_FRAMEBUFFER      = 0x8CA8;
constexpr GLenum DRAW_FRAMEBUFFER      = 0x8CA9;
constexpr GLenum FRAMEBUFFER           = 0x8D40;
constexpr GLenum RENDERBUFFER          = 0x8D41;
constexpr GLenum COLOR_ATTACHMENT0     = 0x8CE0;
constexpr GLenum DEPTH_ATTACHMENT      = 0x8D00;
constexpr GLenum DEPTH_COMPONENT24     = 0x81A6;
constexpr GLenum RGBA8                 = 0x8058;
constexpr GLenum FRAMEBUFFER_COMPLETE  = 0x8CD5;
constexpr GLenum BACK                  = 0x0405;
constexpr GLenum FRONT                 = 0x0404;

// X-macro list of the functions the viewer uses.
#define MESHREPAIR_GL_FUNCTIONS(X)                                                                                    \
    X(void, Clear, (GLbitfield mask))                                                                                 \
    X(void, ClearColor, (GLfloat r, GLfloat g, GLfloat b, GLfloat a))                                                 \
    X(void, Viewport, (GLint x, GLint y, GLsizei w, GLsizei h))                                                       \
    X(void, Enable, (GLenum cap))                                                                                     \
    X(void, Disable, (GLenum cap))                                                                                    \
    X(void, DepthFunc, (GLenum func))                                                                                 \
    X(void, DepthMask, (GLboolean flag))                                                                              \
    X(void, BlendFunc, (GLenum sfactor, GLenum dfactor))                                                              \
    X(void, PolygonOffset, (GLfloat factor, GLfloat units))                                                           \
    X(void, LineWidth, (GLfloat width))                                                                               \
    X(void, ReadPixels, (GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void* data))             \
    X(void, PixelStorei, (GLenum pname, GLint param))                                                                 \
    X(GLenum, GetError, ())                                                                                           \
    X(void, GenVertexArrays, (GLsizei n, GLuint * arrays))                                                            \
    X(void, BindVertexArray, (GLuint array))                                                                          \
    X(void, DeleteVertexArrays, (GLsizei n, const GLuint* arrays))                                                    \
    X(void, GenBuffers, (GLsizei n, GLuint * buffers))                                                                \
    X(void, BindBuffer, (GLenum target, GLuint buffer))                                                               \
    X(void, BufferData, (GLenum target, GLsizeiptr size, const void* data, GLenum usage))                             \
    X(void, DeleteBuffers, (GLsizei n, const GLuint* buffers))                                                        \
    X(void, VertexAttribPointer,                                                                                      \
      (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer))            \
    X(void, EnableVertexAttribArray, (GLuint index))                                                                  \
    X(GLuint, CreateShader, (GLenum type))                                                                            \
    X(void, ShaderSource, (GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length))           \
    X(void, CompileShader, (GLuint shader))                                                                           \
    X(void, GetShaderiv, (GLuint shader, GLenum pname, GLint * params))                                              \
    X(void, GetShaderInfoLog, (GLuint shader, GLsizei bufSize, GLsizei * length, GLchar * infoLog))                  \
    X(void, DeleteShader, (GLuint shader))                                                                            \
    X(GLuint, CreateProgram, ())                                                                                      \
    X(void, AttachShader, (GLuint program, GLuint shader))                                                            \
    X(void, BindAttribLocation, (GLuint program, GLuint index, const GLchar* name))                                  \
    X(void, LinkProgram, (GLuint program))                                                                            \
    X(void, GetProgramiv, (GLuint program, GLenum pname, GLint * params))                                            \
    X(void, GetProgramInfoLog, (GLuint program, GLsizei bufSize, GLsizei * length, GLchar * infoLog))                \
    X(void, DeleteProgram, (GLuint program))                                                                          \
    X(void, UseProgram, (GLuint program))                                                                             \
    X(GLint, GetUniformLocation, (GLuint program, const GLchar* name))                                               \
    X(void, UniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat* value))            \
    X(void, Uniform1f, (GLint location, GLfloat v0))                                                                  \
    X(void, Uniform1i, (GLint location, GLint v0))                                                                    \
    X(void, Uniform3f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2))                                          \
    X(void, Uniform4f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3))                              \
    X(void, DrawArrays, (GLenum mode, GLint first, GLsizei count))                                                    \
    X(void, GenFramebuffers, (GLsizei n, GLuint * framebuffers))                                                      \
    X(void, BindFramebuffer, (GLenum target, GLuint framebuffer))                                                     \
    X(void, DeleteFramebuffers, (GLsizei n, const GLuint* framebuffers))                                              \
    X(void, GenRenderbuffers, (GLsizei n, GLuint * renderbuffers))                                                    \
    X(void, BindRenderbuffer, (GLenum target, GLuint renderbuffer))                                                   \
    X(void, DeleteRenderbuffers, (GLsizei n, const GLuint* renderbuffers))                                            \
    X(void, RenderbufferStorage, (GLenum target, GLenum internalformat, GLsizei width, GLsizei height))              \
    X(void, FramebufferRenderbuffer,                                                                                  \
      (GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer))                            \
    X(GLenum, CheckFramebufferStatus, (GLenum target))                                                                \
    X(void, ReadBuffer, (GLenum mode))                                                                                \
    X(void, CullFace, (GLenum mode))

#define MESHREPAIR_GL_DECLARE(ret, name, args)                                                                        \
    using PFN_##name = ret(*) args;                                                                                   \
    extern PFN_##name name;
MESHREPAIR_GL_FUNCTIONS(MESHREPAIR_GL_DECLARE)
#undef MESHREPAIR_GL_DECLARE

using GetProcAddress = void* (*)(const char* name);

// Resolves all functions. Returns the name of the first missing function, or
// nullptr on success.
const char* load(GetProcAddress get_proc);

} // namespace gl
