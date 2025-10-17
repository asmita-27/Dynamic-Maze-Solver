#include "SimpleRenderer.h"
#include <windows.h>
#include <GL/gl.h>
#include <cmath>
#include <iostream>
#include <sstream>

// Global for text rendering
static HDC g_hDC = nullptr;

SimpleRenderer::SimpleRenderer() : width_(800), height_(600), cellSize_(20.0f), 
                                  offsetX_(50.0f), offsetY_(50.0f) {
}

void SimpleRenderer::setDC(HDC hDC) {
    g_hDC = hDC;
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
    // Dynamically calculate cell size based on grid dimensions
    float gridWidth = grid.getWidth();
    float gridHeight = grid.getHeight();
    
    // Reserve space on the right for instructions (300px)
    float instructionWidth = 300.0f;
    float availableWidth = width_ - instructionWidth - 100.0f;  // Extra padding
    float availableHeight = height_ - 100.0f;
    
    float cellWidth = availableWidth / gridWidth;
    float cellHeight = availableHeight / gridHeight;
    
    // Use the smaller dimension to keep cells square
    cellSize_ = (cellWidth < cellHeight) ? cellWidth : cellHeight;
    
    // Center the grid in the available space
    float gridTotalWidth = gridWidth * cellSize_;
    float gridTotalHeight = gridHeight * cellSize_;
    offsetX_ = (availableWidth - gridTotalWidth) / 2.0f + 50.0f;
    offsetY_ = (availableHeight - gridTotalHeight) / 2.0f + 50.0f;
    
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
    
    // Render path as blue squares (lighter blue for visibility)
    setColor(0.3f, 0.6f, 1.0f); // Light blue
    
    for (const auto& point : path) {
        float centerX = offsetX_ + point.x * cellSize_ + cellSize_ / 2;
        float centerY = offsetY_ + point.y * cellSize_ + cellSize_ / 2;
        float size = cellSize_ * 0.3f;
        
        drawRect(centerX - size/2, centerY - size/2, size, size);
    }
    
    // Render path connections as lines
    if (path.size() > 1) {
        setColor(0.0f, 0.4f, 0.9f); // Darker blue
        glLineWidth(3.0f);
        
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
    
    // Draw agent as yellow circle
    setColor(1.0f, 0.9f, 0.0f); // Bright yellow
    drawCircle(centerX, centerY, radius);
    
    // Draw agent border
    setColor(0.8f, 0.6f, 0.0f); // Darker yellow/orange
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

void SimpleRenderer::renderText(const std::string& text, float x, float y, int size, COLORREF color) {
    if (!g_hDC) return;
    
    // Save OpenGL state
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    
    // Setup font - Use formal, professional font
    HFONT hFont = CreateFont(
        size, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, VARIABLE_PITCH | FF_SWISS, TEXT("Segoe UI")
    );
    
    HFONT hOldFont = (HFONT)SelectObject(g_hDC, hFont);
    
    // Set text properties with anti-aliasing
    SetTextColor(g_hDC, color);
    SetBkMode(g_hDC, TRANSPARENT);
    SetTextAlign(g_hDC, TA_LEFT | TA_TOP);
    
    // Draw text with better quality
    TextOutA(g_hDC, (int)x, (int)y, text.c_str(), text.length());
    
    // Restore
    SelectObject(g_hDC, hOldFont);
    DeleteObject(hFont);
    
    glPopAttrib();
}

void SimpleRenderer::renderInstructions(int gameState, bool manualMode, int level) {
    if (!g_hDC) return;
    
    int startY = 15;
    int lineHeight = 25;
    int textX = width_ - 280;
    
    // Title - always show level (except on main menu)
    if (gameState != 0) {
        std::stringstream ss;
        ss << "LEVEL " << level << "/5";
        renderText(ss.str(), textX, startY, 22, RGB(255, 255, 0));
    }
    
    if (gameState == 0) { // MAIN_MENU - Info and start button
        int centerX = width_ / 2;
        int centerY = 100;
        
        // Title area text
        renderText("DYNAMIC MAZE SOLVER", centerX - 180, centerY, 28, RGB(255, 255, 100));
        
        // Left panel - Controls info
        renderText("CONTROLS:", 150, 180, 20, RGB(255, 255, 100));
        renderText("MOUSE - Click buttons", 150, 215, 16, RGB(200, 200, 200));
        renderText("MOUSE - Select start/goal", 150, 240, 16, RGB(200, 200, 200));
        renderText("MOUSE - Toggle obstacles", 150, 265, 16, RGB(200, 200, 200));
        renderText("ENTER - Move one step", 150, 290, 16, RGB(200, 200, 200));
        renderText("R - Reset to menu", 150, 315, 16, RGB(200, 200, 200));
        
        // Right panel - Game info
        renderText("GAME INFO:", 600, 180, 20, RGB(255, 255, 100));
        renderText("5 Difficulty Levels", 600, 215, 16, RGB(200, 200, 200));
        renderText("10x10 to 30x30 grids", 600, 240, 16, RGB(200, 200, 200));
        renderText("A* & Dynamic A*", 600, 265, 16, RGB(200, 200, 200));
        renderText("Path recalculation", 600, 290, 16, RGB(200, 200, 200));
        renderText("Progressive challenge", 600, 315, 16, RGB(200, 200, 200));
        
        // Start button text
        renderText("CLICK TO START", centerX - 80, 470, 20, RGB(255, 255, 255));
        
    } else if (gameState == 1) { // MODE_SELECTION
        int centerX = width_ / 2;
        renderText("SELECT MODE", centerX - 100, 100, 24, RGB(255, 255, 100));
        
        renderText("MANUAL MODE", 260, 290, 18, RGB(255, 255, 255));
        renderText("Click to toggle obstacles", 220, 315, 14, RGB(200, 200, 200));
        
        renderText("AUTO MODE", 615, 290, 18, RGB(255, 255, 255));
        renderText("Random obstacles each move", 570, 315, 14, RGB(200, 200, 200));
        
    } else if (gameState == 2) { // LEVEL_SELECTION
        int centerX = width_ / 2;
        renderText("SELECT DIFFICULTY", centerX - 130, 90, 24, RGB(255, 255, 100));
        
        std::string levelNames[] = {
            "EASY - 10x10 Grid",
            "MEDIUM - 15x15 Grid",
            "HARD - 20x20 Grid",
            "VERY HARD - 25x25 Grid",
            "EXPERT - 30x30 Grid"
        };
        
        float startY = 180;
        for (int i = 0; i < 5; i++) {
            renderText(levelNames[i], 330, static_cast<int>(startY + i * 80 + 22), 16, RGB(255, 255, 255));
        }
        
    } else if (gameState == 3) { // SELECTING_START
        renderText("Click on grid to", textX, startY + lineHeight * 2, 18, RGB(255, 255, 255));
        renderText("SELECT START", textX, startY + lineHeight * 3, 20, RGB(100, 255, 100));
        renderText("(Green position)", textX, startY + lineHeight * 4, 16, RGB(180, 180, 180));
        
    } else if (gameState == 4) { // SELECTING_GOAL
        renderText("Click on grid to", textX, startY + lineHeight * 2, 18, RGB(255, 255, 255));
        renderText("SELECT GOAL", textX, startY + lineHeight * 3, 20, RGB(255, 100, 100));
        renderText("(Red position)", textX, startY + lineHeight * 4, 16, RGB(180, 180, 180));
        
    } else if (gameState == 5) { // PLAYING
        if (manualMode) {
            renderText("MANUAL MODE", textX, startY + lineHeight * 2, 18, RGB(100, 255, 100));
        } else {
            renderText("AUTO MODE", textX, startY + lineHeight * 2, 18, RGB(255, 150, 150));
        }
        
        renderText("Controls:", textX, startY + lineHeight * 3 + 10, 16, RGB(255, 255, 255));
        renderText("ENTER - Move step", textX, startY + lineHeight * 4 + 10, 15, RGB(255, 255, 100));
        
        if (manualMode) {
            renderText("CLICK - Toggle", textX, startY + lineHeight * 5 + 5, 15, RGB(150, 200, 255));
            renderText("        obstacle", textX, startY + lineHeight * 5 + 22, 14, RGB(180, 180, 180));
        } else {
            renderText("(Obstacles change", textX, startY + lineHeight * 5 + 5, 14, RGB(255, 150, 150));
            renderText(" after each move)", textX, startY + lineHeight * 5 + 22, 14, RGB(255, 150, 150));
        }
        
        renderText("R - Reset", textX, startY + lineHeight * 6 + 30, 14, RGB(200, 200, 200));
        renderText("ESC - Quit", textX, startY + lineHeight * 6 + 50, 14, RGB(200, 200, 200));
        
    } else if (gameState == 6) { // LEVEL_COMPLETE
        renderText("GOAL REACHED!", textX, startY + lineHeight * 2, 20, RGB(100, 255, 100));
        renderText("Great Job!", textX, startY + lineHeight * 3 + 10, 18, RGB(255, 255, 100));
        renderText("Press N for", textX, startY + lineHeight * 4 + 15, 16, RGB(255, 255, 255));
        renderText("Next Level", textX, startY + lineHeight * 5 + 15, 16, RGB(255, 255, 255));
        renderText("or R to Reset", textX, startY + lineHeight * 6 + 15, 14, RGB(200, 200, 200));
    }
    
    // Legend at bottom right - only show during gameplay
    if (gameState >= 3 && gameState <= 6) {
        int legendY = height_ - 120;
        renderText("Legend:", textX, legendY, 16, RGB(200, 200, 200));
        renderText("Green = Start", textX, legendY + 22, 14, RGB(100, 255, 100));
        renderText("Red = Goal", textX, legendY + 38, 14, RGB(255, 100, 100));
        renderText("Blue = Path", textX, legendY + 54, 14, RGB(100, 150, 255));
        renderText("Yellow = Agent", textX, legendY + 70, 14, RGB(255, 255, 100));
        renderText("Black = Obstacle", textX, legendY + 86, 14, RGB(150, 150, 150));
    }
}

