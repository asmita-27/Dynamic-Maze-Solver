#pragma once
#include "Point.h"
#include "Grid.h"
#include <vector>
#include <string>
#include <windows.h>

class SimpleRenderer {
public:
    SimpleRenderer();
    
    bool initialize();
    void clear(float r = 0.1f, float g = 0.15f, float b = 0.2f);
    void renderGrid(const Grid& grid);
    void renderAgent(const Point& position);
    void renderPath(const std::vector<Point>& path);
    void renderText(const std::string& text, float x, float y, int size = 16, COLORREF color = RGB(255, 255, 255));
    void renderInstructions(int gameState, bool manualMode, int level);
    void resize(int width, int height);
    void setDC(HDC hDC);
    
    Point screenToGrid(int screenX, int screenY) const;

private:
    int width_, height_;
    mutable float cellSize_;  // Mutable so it can be updated in const methods
    mutable float offsetX_, offsetY_;
    
    void drawRect(float x, float y, float w, float h);
    void drawCircle(float x, float y, float radius);
    void setColor(float r, float g, float b);
};
