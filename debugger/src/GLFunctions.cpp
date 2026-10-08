/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/GLFunctions.hpp"

namespace Baryon::Debugger::GL {

PFNGLGENVERTEXARRAYSPROC glGenVertexArrays = nullptr;
PFNGLBINDVERTEXARRAYPROC glBindVertexArray = nullptr;
PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays = nullptr;

PFNGLGENBUFFERSPROC glGenBuffers = nullptr;
PFNGLBINDBUFFERPROC glBindBuffer = nullptr;
PFNGLBUFFERDATAPROC glBufferData = nullptr;
PFNGLDELETEBUFFERSPROC glDeleteBuffers = nullptr;

PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray = nullptr;
PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer = nullptr;

PFNGLCREATESHADERPROC glCreateShader = nullptr;
PFNGLSHADERSOURCEPROC glShaderSource = nullptr;
PFNGLCOMPILESHADERPROC glCompileShader = nullptr;
PFNGLGETSHADERIVPROC glGetShaderiv = nullptr;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog = nullptr;
PFNGLDELETESHADERPROC glDeleteShader = nullptr;

PFNGLCREATEPROGRAMPROC glCreateProgram = nullptr;
PFNGLATTACHSHADERPROC glAttachShader = nullptr;
PFNGLLINKPROGRAMPROC glLinkProgram = nullptr;
PFNGLGETPROGRAMIVPROC glGetProgramiv = nullptr;
PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog = nullptr;
PFNGLUSEPROGRAMPROC glUseProgram = nullptr;
PFNGLDELETEPROGRAMPROC glDeleteProgram = nullptr;

PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation = nullptr;
PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv = nullptr;

PFNGLGENFRAMEBUFFERSPROC glGenFramebuffers = nullptr;
PFNGLBINDFRAMEBUFFERPROC glBindFramebuffer = nullptr;
PFNGLFRAMEBUFFERTEXTURE2DPROC glFramebufferTexture2D = nullptr;
PFNGLDELETEFRAMEBUFFERSPROC glDeleteFramebuffers = nullptr;
PFNGLCHECKFRAMEBUFFERSTATUSPROC glCheckFramebufferStatus = nullptr;

PFNGLGENRENDERBUFFERSPROC glGenRenderbuffers = nullptr;
PFNGLBINDRENDERBUFFERPROC glBindRenderbuffer = nullptr;
PFNGLRENDERBUFFERSTORAGEPROC glRenderbufferStorage = nullptr;
PFNGLFRAMEBUFFERRENDERBUFFERPROC glFramebufferRenderbuffer = nullptr;
PFNGLDELETERENDERBUFFERSPROC glDeleteRenderbuffers = nullptr;

#define LOAD_PROC(type, name) name = reinterpret_cast<type>(procAddress(#name))

bool initFunctions(void* (*procAddress)(const char*)) {
    LOAD_PROC(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays);
    LOAD_PROC(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray);
    LOAD_PROC(PFNGLDELETEVERTEXARRAYSPROC, glDeleteVertexArrays);

    LOAD_PROC(PFNGLGENBUFFERSPROC, glGenBuffers);
    LOAD_PROC(PFNGLBINDBUFFERPROC, glBindBuffer);
    LOAD_PROC(PFNGLBUFFERDATAPROC, glBufferData);
    LOAD_PROC(PFNGLDELETEBUFFERSPROC, glDeleteBuffers);

    LOAD_PROC(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray);
    LOAD_PROC(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer);

    LOAD_PROC(PFNGLCREATESHADERPROC, glCreateShader);
    LOAD_PROC(PFNGLSHADERSOURCEPROC, glShaderSource);
    LOAD_PROC(PFNGLCOMPILESHADERPROC, glCompileShader);
    LOAD_PROC(PFNGLGETSHADERIVPROC, glGetShaderiv);
    LOAD_PROC(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog);
    LOAD_PROC(PFNGLDELETESHADERPROC, glDeleteShader);

    LOAD_PROC(PFNGLCREATEPROGRAMPROC, glCreateProgram);
    LOAD_PROC(PFNGLATTACHSHADERPROC, glAttachShader);
    LOAD_PROC(PFNGLLINKPROGRAMPROC, glLinkProgram);
    LOAD_PROC(PFNGLGETPROGRAMIVPROC, glGetProgramiv);
    LOAD_PROC(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog);
    LOAD_PROC(PFNGLUSEPROGRAMPROC, glUseProgram);
    LOAD_PROC(PFNGLDELETEPROGRAMPROC, glDeleteProgram);

    LOAD_PROC(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation);
    LOAD_PROC(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv);

    LOAD_PROC(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers);
    LOAD_PROC(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer);
    LOAD_PROC(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D);
    LOAD_PROC(PFNGLDELETEFRAMEBUFFERSPROC, glDeleteFramebuffers);
    LOAD_PROC(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus);

    LOAD_PROC(PFNGLGENRENDERBUFFERSPROC, glGenRenderbuffers);
    LOAD_PROC(PFNGLBINDRENDERBUFFERPROC, glBindRenderbuffer);
    LOAD_PROC(PFNGLRENDERBUFFERSTORAGEPROC, glRenderbufferStorage);
    LOAD_PROC(PFNGLFRAMEBUFFERRENDERBUFFERPROC, glFramebufferRenderbuffer);
    LOAD_PROC(PFNGLDELETERENDERBUFFERSPROC, glDeleteRenderbuffers);

    return (glGenVertexArrays && glGenBuffers && glCreateProgram && glGenFramebuffers);
}

} // namespace Baryon::Debugger::GL
