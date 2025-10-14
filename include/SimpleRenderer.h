#pragma once
#include "Point.h"
#include "Grid.h"
#include <vector>

class SimpleRenderer {
public:
    SimpleRenderer();
    
    bool initialize();
    void clear(float r = 0.1f, float g = 0.15f, float b = 0.2f);
    void renderGrid(const Grid& grid);
    void renderAgent(const Point& position);
    void renderPath(const std::vector<Point>& path);
    void resize(int width, int height);
    
    Point screenToGrid(int screenX, int screenY) const;

private:
    int width_, height_;
    float cellSize_;
    float offsetX_, offsetY_;
    
    void drawRect(float x, float y, float w, float h);
    void drawCircle(float x, float y, float radius);
    void setColor(float r, float g, float b);
};
