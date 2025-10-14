#include "SimpleRenderer.h"
#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <iostream>

SimpleRenderer::SimpleRenderer() : width_(800), height_(600), cellSize_(20.0f), 
                                  offsetX_(50.0f), offsetY_(50.0f) {
}

bool SimpleRenderer::initialize() {
    glViewport(0, 0, width_, height_);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width_, height_, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    std::cout << "SimpleRenderer initialized: " << width_ << "x" << height_ << std::endl;
    return true;
}

void SimpleRenderer::clear(float r, float g, float b) {
    glClearColor(r, g, b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void SimpleRenderer::renderGrid(const Grid& grid) {
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            Point pos(x, y);
            CellType type = grid.getCellType(pos);
            
            float left = offsetX_ + x * cellSize_;
            float top = offsetY_ + y * cellSize_;
            float size = cellSize_ - 1.0f;
            
            // Set color based on cell type
            switch (type) {
                case CellType::FREE:
                    setColor(0.95f, 0.95f, 0.95f); // Light gray
                    break;
                case CellType::OBSTACLE:
                    setColor(0.1f, 0.1f, 0.1f); // Dark gray/black
                    break;
                case CellType::START:
                    setColor(0.2f, 1.0f, 0.2f); // Green
                    break;
                case CellType::GOAL:
                    setColor(1.0f, 0.2f, 0.2f); // Red
                    break;
            }
            
            // Draw filled rectangle
            drawRect(left, top, size, size);
            
            // Draw border
            setColor(0.7f, 0.7f, 0.7f);
            glLineWidth(1.0f);
            glBegin(GL_LINE_LOOP);
            glVertex2f(left, top);
            glVertex2f(left + size, top);
            glVertex2f(left + size, top + size);
            glVertex2f(left, top + size);
            glEnd();
        }
    }
}

void SimpleRenderer::renderPath(const std::vector<Point>& path) {
    if (path.size() < 2) return;
    
    // Render path as yellow squares
    setColor(1.0f, 1.0f, 0.0f); // Yellow
    
    for (const auto& point : path) {
        float centerX = offsetX_ + point.x * cellSize_ + cellSize_ / 2;
        float centerY = offsetY_ + point.y * cellSize_ + cellSize_ / 2;
        float size = cellSize_ * 0.4f;
        
        drawRect(centerX - size/2, centerY - size/2, size, size);
    }
    
    // Render path connections as lines
    if (path.size() > 1) {
        setColor(1.0f, 0.8f, 0.0f); // Slightly darker yellow
        glLineWidth(2.0f);
        
        glBegin(GL_LINE_STRIP);
        for (const auto& point : path) {
            float centerX = offsetX_ + point.x * cellSize_ + cellSize_ / 2;
            float centerY = offsetY_ + point.y * cellSize_ + cellSize_ / 2;
            glVertex2f(centerX, centerY);
        }
        glEnd();
        
        glLineWidth(1.0f);
    }
}

void SimpleRenderer::renderAgent(const Point& position) {
    float centerX = offsetX_ + position.x * cellSize_ + cellSize_ / 2;
    float centerY = offsetY_ + position.y * cellSize_ + cellSize_ / 2;
    float radius = cellSize_ * 0.4f;
    
    // Draw agent as blue circle
    setColor(0.2f, 0.5f, 1.0f); // Blue
    drawCircle(centerX, centerY, radius);
    
    // Draw agent border
    setColor(0.0f, 0.2f, 0.8f); // Darker blue
    glLineWidth(2.0f);
    
    glBegin(GL_LINE_LOOP);
    const int segments = 16;
    for (int i = 0; i < segments; i++) {
        float angle = 2.0f * 3.14159f * i / segments;
        float x = centerX + radius * cosf(angle);
        float y = centerY + radius * sinf(angle);
        glVertex2f(x, y);
    }
    glEnd();
    
    glLineWidth(1.0f);
}

void SimpleRenderer::resize(int width, int height) {
    if (width <= 0 || height <= 0) return;
    
    width_ = width;
    height_ = height;
    
    glViewport(0, 0, width_, height_);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width_, height_, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    std::cout << "SimpleRenderer resized to: " << width_ << "x" << height_ << std::endl;
}

Point SimpleRenderer::screenToGrid(int screenX, int screenY) const {
    int gridX = static_cast<int>((screenX - offsetX_) / cellSize_);
    int gridY = static_cast<int>((screenY - offsetY_) / cellSize_);
    
    return Point(gridX, gridY);
}

void SimpleRenderer::drawRect(float x, float y, float w, float h) {
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();
}

void SimpleRenderer::drawCircle(float x, float y, float radius) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y); // Center
    
    const int segments = 16;
    for (int i = 0; i <= segments; i++) {
        float angle = 2.0f * 3.14159f * i / segments;
        float px = x + radius * cosf(angle);
        float py = y + radius * sinf(angle);
        glVertex2f(px, py);
    }
    glEnd();
}

void SimpleRenderer::setColor(float r, float g, float b) {
    glColor3f(r, g, b);
}
