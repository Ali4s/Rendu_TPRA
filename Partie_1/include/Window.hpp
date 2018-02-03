#pragma once

#ifndef WINDOW_HPP
#define WINDOW_HPP

#include "GLFW/glfw3.h"
//#include "ArcV/Processing/Image.hpp"


    class Window {
    public:
        Window(unsigned int width, unsigned int height, const std::string &name = "");

        //void mapImage(const Matrix <uint8_t> &mat) const;

        bool show() const;

        void close() const;

        ~Window() { close(); }

    private:
        GLFWwindow *window;

        GLuint vaoIndex;
        GLuint vboIndex;
        GLuint eboIndex;
        GLuint shaderProgram;
        GLuint textureIndex;

        void DrawCube()const;
    };


#endif // WINDOW_HPP
