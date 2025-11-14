#include <windows.h>
#include <GL/gl.h>
#include <iostream>
#include <memory>

#include "Config.h"
#include "Grid.h"
#include "Agent.h"
#include "SimpleRenderer.h"

// Global components
std::unique_ptr<Config> g_config;
std::unique_ptr<Grid> g_grid;
std::unique_ptr<Agent> g_agent;
std::unique_ptr<SimpleRenderer> g_renderer;

// Game state
enum class GameState {
    MAIN_MENU,          // New: Shows controls, levels, info
    MODE_SELECTION,      // Choose manual or auto
    LEVEL_SELECTION,     // Choose difficulty level
    SELECTING_START,
    SELECTING_GOAL,
    PLAYING,
    LEVEL_COMPLETE
};

// Button structure for clickable UI
struct Button {
    float x, y, width, height;
    std::string label;
    int value;  // For storing level number or mode type
    
    bool contains(int mouseX, int mouseY) const {
        return mouseX >= x && mouseX <= x + width &&
               mouseY >= y && mouseY <= y + height;
    }
};

// Global variables for menu state
int g_currentLevel = 1;
const int MAX_LEVEL = 5;
bool g_manualObstacleMode = true;
GameState g_gameState = GameState::MAIN_MENU;
Point g_selectedStart(-1, -1);
Point g_selectedGoal(-1, -1);

// Menu buttons
std::vector<Button> g_menuButtons;
std::vector<Button> g_modeButtons;
std::vector<Button> g_levelButtons;

// Window variables
HWND g_hwnd = nullptr;
HDC g_hdc = nullptr;
HGLRC g_hglrc = nullptr;

// Font display lists for OpenGL bitmap fonts
GLuint g_fontBase = 0;

// Timing
DWORD g_lastUpdateTime = 0;
DWORD g_lastRenderTime = 0;
DWORD g_lastReplanTime = 0;  // For debouncing path recalculation
bool g_needsRender = true; // Flag to control rendering
bool g_needsReplan = false; // Flag for delayed replanning

// Loop detection for AUTO mode
std::vector<Point> g_recentPositions;
const int MAX_POSITION_HISTORY = 8;  // Track last 8 positions
int g_loopDetectionCount = 0;
bool g_allowManualToggleInAuto = false;  // Allow manual obstacle toggle when stuck
bool g_showComparator = true;

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

// Function to detect if agent is stuck in a loop
bool detectLoop(const Point& newPos) {
    // Add new position to history
    g_recentPositions.push_back(newPos);
    if (g_recentPositions.size() > MAX_POSITION_HISTORY) {
        g_recentPositions.erase(g_recentPositions.begin());
    }
    
    // Check if we're revisiting positions frequently (possible loop)
    if (g_recentPositions.size() >= 6) {
        int revisitCount = 0;
        for (const auto& pos : g_recentPositions) {
            if (pos == newPos) {
                revisitCount++;
            }
        }
        
        // If we've been to this position 3+ times in recent history, likely a loop
        if (revisitCount >= 3) {
            g_loopDetectionCount++;
            
            // If we detect loops multiple times, enable manual control
            if (g_loopDetectionCount >= 2 && !g_manualObstacleMode) {
                g_allowManualToggleInAuto = true;
                std::cout << "\n⚠️  LOOP DETECTED! You appear to be stuck." << std::endl;
                std::cout << "💡 Press SPACEBAR to toggle obstacles manually and break free!" << std::endl;
                std::cout << "   Click on cells to add/remove obstacles." << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool InitializeOpenGL() {
    g_hdc = GetDC(g_hwnd);
    if (!g_hdc) {
        std::cerr << "Failed to get device context" << std::endl;
        return false;
    }
    
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 16;
    
    int pixelFormat = ChoosePixelFormat(g_hdc, &pfd);
    if (!pixelFormat) {
        std::cerr << "Failed to choose pixel format" << std::endl;
        return false;
    }
    
    if (!SetPixelFormat(g_hdc, pixelFormat, &pfd)) {
        std::cerr << "Failed to set pixel format" << std::endl;
        return false;
    }
    
    g_hglrc = wglCreateContext(g_hdc);
    if (!g_hglrc) {
        std::cerr << "Failed to create OpenGL context" << std::endl;
        return false;
    }
    
    if (!wglMakeCurrent(g_hdc, g_hglrc)) {
        std::cerr << "Failed to make OpenGL context current" << std::endl;
        return false;
    }
    
    // Enable VSync to eliminate tearing and flickering
    typedef BOOL (WINAPI *PFNWGLSWAPINTERVALEXTPROC)(int interval);
    PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT = 
        (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
    if (wglSwapIntervalEXT) {
        wglSwapIntervalEXT(1); // Enable VSync (1 = wait for vsync)
        std::cout << "VSync enabled - smoother rendering" << std::endl;
    }
    
    std::cout << "OpenGL initialized successfully" << std::endl;
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    
    return true;
}

bool CreateGameWindow() {
    const char* CLASS_NAME = "DynamicMazeSolver";
    
    // Register window class
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = CLASS_NAME;
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.style = CS_OWNDC;
    
    if (!RegisterClass(&wc)) {
        std::cerr << "Failed to register window class" << std::endl;
        return false;
    }
    
    // Create window
    g_hwnd = CreateWindowEx(
        0,
        CLASS_NAME,
        "Dynamic Maze Solver - A* and Dynamic A* (D* Lite)",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1000, 800,
        nullptr, nullptr, GetModuleHandle(nullptr), nullptr
    );
    
    if (!g_hwnd) {
        std::cerr << "Failed to create window" << std::endl;
        return false;
    }
    
    ShowWindow(g_hwnd, SW_SHOW);
    UpdateWindow(g_hwnd);
    
    return true;
}

void Update() {
    DWORD currentTime = GetTickCount();
    
    // Only update during playing state
    if (g_gameState == GameState::PLAYING && g_agent && g_grid) {
        // Debounced path replanning (only every 200ms to avoid lag)
        if (g_needsReplan && (currentTime - g_lastReplanTime) >= 200) {
            g_agent->planPath(*g_grid);
            g_needsReplan = false;
            g_lastReplanTime = currentTime;
            g_needsRender = true;
        }
        
        // Update agent (handles auto-movement and path replanning)
        g_agent->update(*g_grid);
        
        // Check if goal reached in auto mode
        if (g_agent->hasReachedGoal() && g_gameState == GameState::PLAYING) {
            std::cout << "[★ GOAL REACHED! ★]" << std::endl;
            g_gameState = GameState::LEVEL_COMPLETE;
            g_needsRender = true;
        }
        
        // Process scheduled grid events
        g_grid->processScheduledEvents();
        
        // No longer continuously mark for re-render to prevent flickering
        // Only render when explicitly needed (user actions, replanning, etc.)
    }
    
    g_lastUpdateTime = currentTime;
}

void drawSimpleText(const std::string& text, float x, float y, float scale = 1.0f) {
    if (!g_hdc || text.empty()) return;
    
    // Calculate font size based on scale
    int fontSize = static_cast<int>(18 * scale);
    
    // Create professional font - Arial for clean, formal look
    HFONT hFont = CreateFont(
        -fontSize,                          // Height (negative for better quality)
        0,                                  // Width (0 = default)
        0,                                  // Escapement
        0,                                  // Orientation
        FW_SEMIBOLD,                        // Weight (semi-bold for readability)
        FALSE,                              // Italic
        FALSE,                              // Underline
        FALSE,                              // Strikeout
        ANSI_CHARSET,                       // Character set
        OUT_TT_PRECIS,                      // Output precision
        CLIP_DEFAULT_PRECIS,                // Clipping precision
        ANTIALIASED_QUALITY,                // Quality (antialiased for smooth text)
        VARIABLE_PITCH | FF_SWISS,          // Pitch and family (Swiss = sans-serif like Arial)
        TEXT("Arial")                       // Font name - professional and clean
    );
    
    if (!hFont) return;
    
    // Select font into device context
    HFONT hOldFont = (HFONT)SelectObject(g_hdc, hFont);
    
    // Create display lists for this font
    GLuint base = glGenLists(256);
    wglUseFontBitmaps(g_hdc, 0, 256, base);
    
    // Save OpenGL state
    glPushAttrib(GL_LIST_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_TRANSFORM_BIT);
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Set text color (white)
    glColor3f(1.0f, 1.0f, 1.0f);
    
    // Set raster position
    glRasterPos2f(x, y + fontSize);  // Offset by font size for proper positioning
    
    // Set list base and render text
    glListBase(base);
    glCallLists(text.length(), GL_UNSIGNED_BYTE, text.c_str());
    
    // Clean up
    glDeleteLists(base, 256);
    glPopAttrib();
    
    // Restore original font
    SelectObject(g_hdc, hOldFont);
    DeleteObject(hFont);
}

void drawButton(const Button& btn, float r, float g, float b, bool hover = false, const std::string& label = "") {
    // Button background
    if (hover) {
        glColor3f(r * 1.3f, g * 1.3f, b * 1.3f); // Brighter on hover
    } else {
        glColor3f(r, g, b);
    }
    glBegin(GL_QUADS);
    glVertex2f(btn.x, btn.y);
    glVertex2f(btn.x + btn.width, btn.y);
    glVertex2f(btn.x + btn.width, btn.y + btn.height);
    glVertex2f(btn.x, btn.y + btn.height);
    glEnd();
    
    // Button border
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(btn.x, btn.y);
    glVertex2f(btn.x + btn.width, btn.y);
    glVertex2f(btn.x + btn.width, btn.y + btn.height);
    glVertex2f(btn.x, btn.y + btn.height);
    glEnd();
    
    // Draw label if provided - properly centered
    if (!label.empty()) {
        glColor3f(1.0f, 1.0f, 1.0f);
        // Approximate character width for Arial at scale 1.5 is about 11 pixels
        float textWidth = label.length() * 11 * 1.5f;
        float textX = btn.x + (btn.width - textWidth) / 2;
        float textY = btn.y + (btn.height / 2) - 8;  // Center vertically
        drawSimpleText(label, textX, textY, 1.5f);
    }
}

void Render() {
    if (!g_renderer) return;
    
    g_renderer->clear(0.1f, 0.15f, 0.2f);
    
    if (g_gameState == GameState::MAIN_MENU) {
        // Title banner - centered
        glColor3f(1.0f, 0.8f, 0.0f);
        glBegin(GL_QUADS);
        glVertex2f(190, 50);
        glVertex2f(810, 50);
        glVertex2f(810, 120);
        glVertex2f(190, 120);
        glEnd();
        
        // Info panels - better spacing and centering
        glColor3f(0.2f, 0.3f, 0.4f);
        // Left panel
        glBegin(GL_QUADS);
        glVertex2f(90, 150);
        glVertex2f(440, 150);
        glVertex2f(440, 360);
        glVertex2f(90, 360);
        glEnd();
        
        // Right panel
        glBegin(GL_QUADS);
        glVertex2f(560, 150);
        glVertex2f(910, 150);
        glVertex2f(910, 360);
        glVertex2f(560, 360);
        glEnd();
        
        // Start button - perfectly centered (500 is center, minus half width)
        Button startBtn = {387, 450, 226, 65, "START", 0};
        drawButton(startBtn, 0.2f, 0.8f, 0.3f, false, "START");
        g_menuButtons.clear();
        g_menuButtons.push_back(startBtn);
        
        // Draw title text - centered
        glColor3f(1.0f, 1.0f, 1.0f);
        drawSimpleText("DYNAMIC MAZE SOLVER", 218, 72, 2.2f);
        
        // Left panel - Controls (centered in panel)
        glColor3f(1.0f, 0.8f, 0.0f);
        drawSimpleText("CONTROLS", 220, 168, 1.4f);
        
        glColor3f(0.9f, 0.9f, 0.9f);
        drawSimpleText("MOUSE - CLICK BUTTONS", 120, 210, 0.85f);
        drawSimpleText("MOUSE - SELECT START AND GOAL", 120, 238, 0.85f);
        drawSimpleText("MOUSE - TOGGLE OBSTACLES", 120, 266, 0.85f);
        drawSimpleText("ARROW KEYS - MOVE AGENT", 120, 294, 0.85f);
        drawSimpleText("M - SWITCH MODE", 120, 322, 0.85f);
        
        // Right panel - Game info (centered in panel)
        glColor3f(1.0f, 0.8f, 0.0f);
        drawSimpleText("GAME INFO", 680, 168, 1.4f);
        
        glColor3f(0.9f, 0.9f, 0.9f);
        drawSimpleText("5 DIFFICULTY LEVELS", 598, 210, 0.85f);
        drawSimpleText("10X10 TO 30X30 GRIDS", 598, 238, 0.85f);
        drawSimpleText("A-STAR AND DYNAMIC A-STAR", 598, 266, 0.85f);
        drawSimpleText("PATHFINDING ALGORITHMS", 598, 294, 0.85f);
        drawSimpleText("PROGRESSIVE CHALLENGE", 598, 322, 0.85f);
        
        static bool logged = false;
        if (!logged) {
            std::cout << "[Rendering MAIN_MENU with start button at: " << startBtn.x << "," << startBtn.y 
                      << " size: " << startBtn.width << "x" << startBtn.height << "]" << std::endl;
            logged = true;
        }
        
    } else if (g_gameState == GameState::MODE_SELECTION) {
        // Title banner - centered
        glColor3f(1.0f, 0.8f, 0.0f);
        glBegin(GL_QUADS);
        glVertex2f(300, 80);
        glVertex2f(700, 80);
        glVertex2f(700, 140);
        glVertex2f(300, 140);
        glEnd();
        
        // Title text - centered
        glColor3f(1.0f, 1.0f, 1.0f);
        drawSimpleText("SELECT MODE", 368, 100, 2.0f);
        
        // Mode buttons - better centered and spaced
        Button manualBtn = {200, 260, 250, 110, "MANUAL", 1};
        Button autoBtn = {550, 260, 250, 110, "AUTO", 2};
        
        drawButton(manualBtn, 0.2f, 0.7f, 0.3f, false, "MANUAL");
        drawButton(autoBtn, 0.7f, 0.3f, 0.3f, false, "AUTO");
        
        g_modeButtons.clear();
        g_modeButtons.push_back(manualBtn);
        g_modeButtons.push_back(autoBtn);
        
    } else if (g_gameState == GameState::LEVEL_SELECTION) {
        // Title banner - centered
        glColor3f(1.0f, 0.8f, 0.0f);
        glBegin(GL_QUADS);
        glVertex2f(250, 60);
        glVertex2f(750, 60);
        glVertex2f(750, 120);
        glVertex2f(250, 120);
        glEnd();
        
        // Title text - centered
        glColor3f(1.0f, 1.0f, 1.0f);
        drawSimpleText("SELECT LEVEL", 355, 80, 2.0f);
        
        // Level buttons
        g_levelButtons.clear();
        float startY = 180;
        float colors[][3] = {
            {0.3f, 0.8f, 0.3f},  // Easy - Green
            {0.5f, 0.7f, 0.3f},  // Medium - Yellow-green
            {0.8f, 0.7f, 0.2f},  // Hard - Orange
            {0.9f, 0.5f, 0.2f},  // Very Hard - Dark orange
            {0.9f, 0.2f, 0.2f}   // Expert - Red
        };
        
        std::string levelNames[] = {"LEVEL 1", "LEVEL 2", "LEVEL 3", "LEVEL 4", "LEVEL 5"};
        
        for (int i = 0; i < MAX_LEVEL; i++) {
            Button levelBtn = {300, startY + i * 80, 400, 60, "", i + 1};
            drawButton(levelBtn, colors[i][0], colors[i][1], colors[i][2], false, levelNames[i]);
            g_levelButtons.push_back(levelBtn);
        }
        
    } else if (g_gameState != GameState::MAIN_MENU && g_gameState != GameState::MODE_SELECTION 
               && g_gameState != GameState::LEVEL_SELECTION && g_grid) {
        // Render grid and game elements
        g_renderer->renderGrid(*g_grid);
        
        if (g_gameState == GameState::SELECTING_START && g_selectedStart.x >= 0) {
            g_renderer->renderAgent(g_selectedStart);
        } else if (g_gameState == GameState::SELECTING_GOAL) {
            if (g_selectedStart.x >= 0) {
                g_renderer->renderAgent(g_selectedStart);
            }
        } else if (g_gameState == GameState::PLAYING || g_gameState == GameState::LEVEL_COMPLETE) {
            if (g_agent) {
                // Only draw paths if agent has a valid path
                if (g_agent->hasPath()) {
                    // Draw comparator paths first (if any), so chosen/current path is highlighted
                    const auto& stats = g_agent->getReplanStats();
                    for (const auto& e : stats) {
                        if (!e.result.success) continue;

                        // Validate path against current grid: skip if any node is blocked
                        bool validPath = true;
                        if (e.result.path.empty()) validPath = false;
                        for (const auto& p : e.result.path) {
                            if (!g_grid->isFree(p)) { validPath = false; break; }
                        }

                        // Also skip paths that don't actually reach the goal (dead-ends)
                        if (validPath) {
                            Point goal = g_agent->getGoal();
                            if (e.result.path.empty() || !(e.result.path.back() == goal)) {
                                validPath = false;
                            }
                        }

                        if (!validPath) continue; // don't render invalid or dead-end paths

                        // A* -> cyan, Dynamic A* -> bright magenta/pink, LPA* -> bright green
                        if (e.name.find("LPA") != std::string::npos) {
                            g_renderer->renderPathColored(e.result.path, 0.0f, 1.0f, 0.3f);
                        } else if (e.name.find("Dynamic") != std::string::npos) {
                            g_renderer->renderPathColored(e.result.path, 0.9f, 0.0f, 0.6f);
                        } else {
                            g_renderer->renderPathColored(e.result.path, 0.4f, 0.7f, 1.0f);
                        }
                    }

                    // Draw the agent's chosen path last so it stands out, but only if it's valid
                    const auto& chosenPath = g_agent->getCurrentPath();
                    bool chosenValid = true;
                    if (chosenPath.empty()) chosenValid = false;
                    for (const auto& p : chosenPath) {
                        if (!g_grid->isFree(p)) { chosenValid = false; break; }
                    }
                    if (chosenValid) {
                        // Also ensure it actually reaches the goal
                        if (!chosenPath.empty() && chosenPath.back() == g_agent->getGoal()) {
                            g_renderer->renderPath(chosenPath);
                        }
                    }
                }
                // Always draw agent position
                g_renderer->renderAgent(g_agent->getPosition());
            }
        }
        
        // Right-side HUD/instructions are now drawn by SimpleRenderer::renderInstructions
        // to avoid duplicate text and overlapping panels. No gameplay-side text
        // is rendered here.
    }
    
    // Render instructions
    g_renderer->renderInstructions(
        static_cast<int>(g_gameState),
        g_manualObstacleMode,
        g_currentLevel,
        g_agent.get()
    );
    
    SwapBuffers(g_hdc);
}

void showMenu() {
    std::cout << "\n==========================================" << std::endl;
    std::cout << "    DYNAMIC MAZE SOLVER" << std::endl;
    std::cout << "    A* and Dynamic A* (D* Lite)" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "\nAll instructions are shown ON the game screen" << std::endl;
    std::cout << "Look at the right side of the game window!" << std::endl;
}

void showLevelInfo(int level) {
    std::cout << "\n========== LEVEL " << level << " ==========" << std::endl;
    switch(level) {
        case 1:
            std::cout << "Difficulty: EASY" << std::endl;
            std::cout << "Grid Size: 10x10" << std::endl;
            std::cout << "Obstacles: 10%" << std::endl;
            break;
        case 2:
            std::cout << "Difficulty: MEDIUM" << std::endl;
            std::cout << "Grid Size: 15x15" << std::endl;
            std::cout << "Obstacles: 15%" << std::endl;
            break;
        case 3:
            std::cout << "Difficulty: HARD" << std::endl;
            std::cout << "Grid Size: 20x20" << std::endl;
            std::cout << "Obstacles: 20%" << std::endl;
            break;
        case 4:
            std::cout << "Difficulty: VERY HARD" << std::endl;
            std::cout << "Grid Size: 25x25" << std::endl;
            std::cout << "Obstacles: 25%" << std::endl;
            break;
        case 5:
            std::cout << "Difficulty: EXPERT" << std::endl;
            std::cout << "Grid Size: 30x30" << std::endl;
            std::cout << "Obstacles: 30%" << std::endl;
            break;
    }
    std::cout << "==================================" << std::endl;
}

void getLevelSettings(int level, int& width, int& height, float& density) {
    switch(level) {
        case 1: width = 10; height = 10; density = 0.10f; break;
        case 2: width = 15; height = 15; density = 0.15f; break;
        case 3: width = 20; height = 20; density = 0.20f; break;
        case 4: width = 25; height = 25; density = 0.25f; break;
        case 5: width = 30; height = 30; density = 0.30f; break;
        default: width = 10; height = 10; density = 0.10f; break;
    }
}

void initializeLevel(int level, bool manualObstacles) {
    int width, height;
    float density;
    getLevelSettings(level, width, height, density);
    
    std::cout << "\n[Level " << level << " initialized - " << width << "x" << height << "]" << std::endl;
    
    // Create new grid
    g_grid = std::make_unique<Grid>(width, height, g_config->isEightDirectional());
    
    // Generate obstacles based on mode and level
    g_grid->generateRandomObstacles(density);
    
    // Reset selections
    g_selectedStart = Point(-1, -1);
    g_selectedGoal = Point(-1, -1);
    
    // Change to selecting start state
    g_gameState = GameState::SELECTING_START;
    
    // Reset loop-detection and manual-override state for a fresh level start
    g_recentPositions.clear();
    g_loopDetectionCount = 0;
    g_allowManualToggleInAuto = false;
    std::cout << "[Loop detection reset for new level]" << std::endl;
    
    std::cout << "[Watch game screen for instructions]" << std::endl;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    // Enable console for output
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
    freopen("CONIN$", "r", stdin);
    freopen("CONERR$", "w", stderr);
    
    showMenu();
    
    // Initialize configuration
    g_config = std::make_unique<Config>();
    g_config->load();
    
    if (!g_config->isValid()) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }
    
    // Create window and initialize OpenGL
    if (!CreateGameWindow()) {
        return 1;
    }
    
    if (!InitializeOpenGL()) {
        return 1;
    }
    
    // Initialize renderer
    g_renderer = std::make_unique<SimpleRenderer>();
    if (!g_renderer->initialize()) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return 1;
    }
    
    // Pass device context to renderer for text rendering
    g_renderer->setDC(g_hdc);
    
    // Set initial window size for renderer
    RECT rect;
    GetClientRect(g_hwnd, &rect);
    g_renderer->resize(rect.right - rect.left, rect.bottom - rect.top);
    std::cout << "Initial window size: " << (rect.right - rect.left) << "x" << (rect.bottom - rect.top) << std::endl;
    
    // Create agent
    g_agent = std::make_unique<Agent>(g_config->getAlgorithm());
    g_agent->setMovementSpeed(g_config->getSimulationSpeed());
    
    // Don't create grid yet - wait for user to choose mode
    // Grid will be created when user presses 1 or 2
    
    std::cout << "\n=== GAME READY ===" << std::endl;
    std::cout << "★ All instructions are shown ON THE GAME SCREEN ★" << std::endl;
    std::cout << "★ Look at the RIGHT SIDE of the window ★" << std::endl;
    std::cout << "\nConsole will show minimal feedback." << std::endl;
    std::cout << "Watch the game window for all controls!" << std::endl;
    
    // Main message loop
    MSG msg = {};
    bool running = true;
    g_lastUpdateTime = GetTickCount();
    g_lastRenderTime = GetTickCount();
    
    while (running) {
        // Process messages
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        
        if (!running) break;
        
        DWORD currentTime = GetTickCount();
        
        // Update at 30 FPS (only when playing)
        if (currentTime - g_lastUpdateTime >= 33) {
            Update();
            g_lastUpdateTime = currentTime;
        }
        
        // Render at 30 FPS (reduced from 60 to prevent flickering)
        if (currentTime - g_lastRenderTime >= 33) {
            // Always render at menu states, or when something changed
            if (g_gameState == GameState::MAIN_MENU || g_gameState == GameState::MODE_SELECTION ||
                g_gameState == GameState::LEVEL_SELECTION || g_needsRender) {
                Render();
                g_needsRender = false;
            }
            g_lastRenderTime = currentTime;
        }
        
        Sleep(5); // Prevent 100% CPU usage and reduce flicker
    }
    
    // Cleanup
    if (g_hglrc) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(g_hglrc);
    }
    if (g_hdc) {
        ReleaseDC(g_hwnd, g_hdc);
    }
    
    FreeConsole();
    
    std::cout << "Application shutting down..." << std::endl;
    return 0;
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
        
    case WM_PAINT: {
        PAINTSTRUCT ps;
        BeginPaint(hwnd, &ps);
        // Rendering is handled in main loop
        EndPaint(hwnd, &ps);
        return 0;
    }
    
    case WM_SIZE:
        if (g_renderer) {
            RECT rect;
            GetClientRect(hwnd, &rect);
            g_renderer->resize(rect.right - rect.left, rect.bottom - rect.top);
        }
        return 0;
        
    case WM_LBUTTONDOWN: {
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);
        
        std::cout << "[Mouse click at: " << x << ", " << y << "]" << std::endl;
        
        // Handle menu button clicks
        if (g_gameState == GameState::MAIN_MENU) {
            std::cout << "[In MAIN_MENU state, checking " << g_menuButtons.size() << " buttons]" << std::endl;
            for (const auto& btn : g_menuButtons) {
                std::cout << "[Button bounds: " << btn.x << "," << btn.y << " to " << (btn.x + btn.width) << "," << (btn.y + btn.height) << "]" << std::endl;
                if (btn.contains(x, y)) {
                    std::cout << "[START clicked - Choose mode]" << std::endl;
                    g_gameState = GameState::MODE_SELECTION;
                    g_needsRender = true;
                    return 0;
                }
            }
            std::cout << "[No button hit]" << std::endl;
        }
        else if (g_gameState == GameState::MODE_SELECTION) {
            std::cout << "[In MODE_SELECTION state, checking " << g_modeButtons.size() << " buttons]" << std::endl;
            for (const auto& btn : g_modeButtons) {
                if (btn.contains(x, y)) {
                    if (btn.value == 1) {
                        g_manualObstacleMode = true;
                        std::cout << "[MANUAL mode selected]" << std::endl;
                    } else {
                        g_manualObstacleMode = false;
                        std::cout << "[AUTO mode selected]" << std::endl;
                    }
                    g_gameState = GameState::LEVEL_SELECTION;
                    g_needsRender = true;
                    return 0;
                }
            }
        }
        else if (g_gameState == GameState::LEVEL_SELECTION) {
            std::cout << "[In LEVEL_SELECTION state, checking " << g_levelButtons.size() << " buttons]" << std::endl;
            for (const auto& btn : g_levelButtons) {
                if (btn.contains(x, y)) {
                    g_currentLevel = btn.value;
                    std::cout << "[Level " << g_currentLevel << " selected]" << std::endl;
                    initializeLevel(g_currentLevel, g_manualObstacleMode);
                    g_needsRender = true;
                    return 0;
                }
            }
        }
        else if (g_renderer) {
            // Handle grid clicks during gameplay
            Point gridPos = g_renderer->screenToGrid(x, y);
            
            if (g_grid && g_grid->inBounds(gridPos)) {
                if (g_gameState == GameState::SELECTING_START) {
                    // Select start position
                    g_selectedStart = gridPos;
                    g_grid->setCell(gridPos, CellType::START);
                    std::cout << "[START: (" << gridPos.x << "," << gridPos.y << ")]" << std::endl;
                    g_gameState = GameState::SELECTING_GOAL;
                    g_needsRender = true;
                    
                } else if (g_gameState == GameState::SELECTING_GOAL) {
                    // Select goal position (must be different from start)
                    if (gridPos == g_selectedStart) {
                        std::cout << "[Error: Goal cannot be same as start]" << std::endl;
                    } else {
                        g_selectedGoal = gridPos;
                        g_grid->setCell(gridPos, CellType::GOAL);
                        std::cout << "[GOAL: (" << gridPos.x << "," << gridPos.y << ")]" << std::endl;
                        
                        // Setup agent and start playing
                        g_agent->setStart(g_selectedStart);
                        g_agent->setGoal(g_selectedGoal);
                        g_agent->setManualMode(g_manualObstacleMode); // FIX: Set agent's mode
                        g_agent->reset();
                        g_agent->planPath(*g_grid);
                        
                        g_gameState = GameState::PLAYING;
                        g_needsRender = true;
                        std::cout << "[Game started - " << (g_manualObstacleMode ? "MANUAL" : "AUTO") << " mode]" << std::endl;
                    }
                    
                } else if (g_gameState == GameState::PLAYING && (g_manualObstacleMode || g_allowManualToggleInAuto)) {
                    // Toggle obstacles in manual mode OR when manual toggle is allowed in auto mode
                    CellType cellType = g_grid->getCellType(gridPos);
                    
                    // Don't toggle start, goal, or agent's current position
                    if (cellType != CellType::START && 
                        cellType != CellType::GOAL && 
                        gridPos != g_agent->getPosition()) {
                        
                        g_grid->toggleObstacle(gridPos);
                        
                        if (g_allowManualToggleInAuto) {
                            std::cout << "[MANUAL OVERRIDE: Toggled obstacle at (" << gridPos.x << "," << gridPos.y << ")]" << std::endl;
                            std::cout << "   Press SPACEBAR again to resume auto mode" << std::endl;
                        } else {
                            std::cout << "[Toggled obstacle at (" << gridPos.x << "," << gridPos.y << ")]" << std::endl;
                        }
                        
                        // Mark for replanning instead of immediate replan (debounced)
                        g_needsReplan = true;
                        g_needsRender = true;
                    }
                }
            }
        }
        return 0;
    }
    
    case WM_KEYDOWN: {
        switch (wParam) {
        case VK_SPACE:
            // Toggle manual control in AUTO mode when stuck in a loop
            if (g_gameState == GameState::PLAYING && !g_manualObstacleMode) {
                g_allowManualToggleInAuto = !g_allowManualToggleInAuto;
                
                if (g_allowManualToggleInAuto) {
                    std::cout << "\n[🔧 MANUAL OVERRIDE ENABLED]" << std::endl;
                    std::cout << "   Click cells to toggle obstacles manually" << std::endl;
                    std::cout << "   Press SPACEBAR again to resume auto mode" << std::endl;
                    // Reset loop detection when enabling manual override
                    g_recentPositions.clear();
                    g_loopDetectionCount = 0;
                } else {
                    std::cout << "\n[🤖 AUTO MODE RESUMED]" << std::endl;
                    std::cout << "   Obstacles will toggle automatically again" << std::endl;
                }
                g_needsRender = true;
            }
            break;
        
        case VK_UP:
        case VK_DOWN:
        case VK_LEFT:
        case VK_RIGHT:
            // Arrow key movement during playing
            if (g_gameState == GameState::PLAYING && g_agent && g_grid) {
                if (g_agent->hasReachedGoal()) {
                    std::cout << "[Already at goal!]" << std::endl;
                    g_gameState = GameState::LEVEL_COMPLETE;
                } else {
                    // Calculate new position based on arrow key
                    Point currentPos = g_agent->getPosition();
                    Point newPos = currentPos;
                    
                    if (wParam == VK_UP) {
                        newPos.y--;  // Move up
                        std::cout << "[Moving UP]" << std::endl;
                    } else if (wParam == VK_DOWN) {
                        newPos.y++;  // Move down
                        std::cout << "[Moving DOWN]" << std::endl;
                    } else if (wParam == VK_LEFT) {
                        newPos.x--;  // Move left
                        std::cout << "[Moving LEFT]" << std::endl;
                    } else if (wParam == VK_RIGHT) {
                        newPos.x++;  // Move right
                        std::cout << "[Moving RIGHT]" << std::endl;
                    }
                    
                    // Check if new position is valid (in bounds and not obstacle)
                    if (g_grid->inBounds(newPos)) {
                        CellType cellType = g_grid->getCellType(newPos);
                        if (cellType != CellType::OBSTACLE) {
                            // Valid move - update agent position
                            g_agent->setStart(newPos);
                            
                            // Detect loops in AUTO mode
                            if (!g_manualObstacleMode) {
                                detectLoop(newPos);
                            }
                            
                            // Force replanning from new position
                            g_agent->forceReplanning();
                            g_agent->planPath(*g_grid);
                            
                            std::cout << "  → Moved to (" << newPos.x << ", " << newPos.y << ")" << std::endl;
                            std::cout << "  → Replanning path from new position..." << std::endl;
                            
                            // In AUTO mode, intelligently toggle obstacles to make path difficult
                            if (!g_manualObstacleMode && !g_agent->hasReachedGoal()) {
                                int width = g_grid->getWidth();
                                int height = g_grid->getHeight();
                                
                                // Get current path
                                const auto& currentPath = g_agent->getCurrentPath();
                                
                                // Progressive difficulty: fewer obstacles in early levels
                                int maxObstacles;
                                switch(g_currentLevel) {
                                    case 1: maxObstacles = 1; break;  // Easy: always 1 obstacle
                                    case 2: maxObstacles = 1; break;  // Medium: always 1 obstacle (not too hard)
                                    case 3: maxObstacles = 2; break;  // Hard: 1-2 obstacles
                                    case 4: maxObstacles = 2; break;  // Very Hard: 1-2 obstacles
                                    case 5: maxObstacles = 3; break;  // Expert: 1-3 obstacles
                                    default: maxObstacles = 2; break;
                                }
                                
                                // For levels 1-2, always use 1 obstacle for easier gameplay
                                int numChanges = (g_currentLevel <= 2) ? 1 : (1 + (rand() % maxObstacles));
                                int successfulChanges = 0;
                                
                                std::cout << "  [AUTO mode Level " << g_currentLevel << ": Toggling " << numChanges << " obstacles]" << std::endl;
                                
                                // Try to place obstacles ahead on the path
                                if (currentPath.size() > 3) {
                                    for (int i = 0; i < numChanges * 3 && successfulChanges < numChanges; i++) {
                                        // Pick a point ahead on the path (not too close, not the goal)
                                        int pathIndex = (rand() % (currentPath.size() - 2)) + 2;
                                        Point targetPos = currentPath[pathIndex];
                                        
                                        // Also try cells adjacent to the path
                                        int dx = (rand() % 3) - 1; // -1, 0, 1
                                        int dy = (rand() % 3) - 1;
                                        Point blockPos(targetPos.x + dx, targetPos.y + dy);
                                        
                                        if (g_grid->inBounds(blockPos)) {
                                            CellType cellType = g_grid->getCellType(blockPos);
                                            
                                            // Don't toggle start, goal, or agent's current position
                                            if (cellType != CellType::START && 
                                                cellType != CellType::GOAL && 
                                                blockPos != g_agent->getPosition()) {
                                                
                                                // Toggle it (turn empty to obstacle or obstacle to empty)
                                                g_grid->toggleObstacle(blockPos);
                                                successfulChanges++;
                                                
                                                if (cellType == CellType::FREE) {
                                                    std::cout << "    → Added obstacle at (" << blockPos.x << "," << blockPos.y << ")" << std::endl;
                                                } else {
                                                    std::cout << "    → Removed obstacle at (" << blockPos.x << "," << blockPos.y << ")" << std::endl;
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    // Fallback: random obstacles if path is very short
                                    for (int i = 0; i < numChanges; i++) {
                                        Point randomPos(rand() % width, rand() % height);
                                        CellType cellType = g_grid->getCellType(randomPos);
                                        
                                        if (cellType != CellType::START && 
                                            cellType != CellType::GOAL && 
                                            randomPos != g_agent->getPosition()) {
                                            g_grid->toggleObstacle(randomPos);
                                        }
                                    }
                                }
                                
                                if (successfulChanges > 0) {
                                    std::cout << "    ⚡ Path blocked! Replanning..." << std::endl;
                                    // Force replanning after obstacles change
                                    g_agent->forceReplanning();
                                    g_agent->planPath(*g_grid);
                                }
                            }
                            
                            // Check if goal reached
                            if (g_agent->hasReachedGoal()) {
                                std::cout << "[★ GOAL REACHED! ★]" << std::endl;
                                g_gameState = GameState::LEVEL_COMPLETE;
                            }
                        } else {
                            std::cout << "  ✗ Cannot move - obstacle in the way!" << std::endl;
                        }
                    } else {
                        std::cout << "  ✗ Cannot move - out of bounds!" << std::endl;
                    }
                    
                    g_needsRender = true;
                }
            }
            break;
        
        case 'N':
        case 'n':
            if (g_gameState == GameState::LEVEL_COMPLETE) {
                if (g_currentLevel < MAX_LEVEL) {
                    g_currentLevel++;
                    std::cout << "[Advancing to Level " << g_currentLevel << " - " << (g_manualObstacleMode ? "MANUAL" : "AUTO") << " mode]" << std::endl;
                    
                    // Reinitialize the level with the same mode
                    initializeLevel(g_currentLevel, g_manualObstacleMode);
                    g_needsRender = true;
                } else {
                    std::cout << "[★ ALL LEVELS COMPLETED! ★]" << std::endl;
                    std::cout << "[You finished all 5 levels! Press R to play again]" << std::endl;
                }
            }
            break;
            
        case 'R':
        case 'r':
            std::cout << "[Reset - Back to main menu]" << std::endl;
            g_gameState = GameState::MAIN_MENU;
            g_needsRender = true;
            break;
            
        case 'A':
        case 'a':
            if (g_agent) {
                g_agent->setPathfinder("AStar");
                std::cout << "🔀 Switched to A* algorithm" << std::endl;
            }
            break;
            
        case 'D':
        case 'd':
            if (g_agent) {
                g_agent->setPathfinder("DynamicAStar");
                std::cout << "🔀 Switched to Dynamic A* algorithm" << std::endl;
            }
            break;
        
        case 'M':
        case 'm':
            // Toggle between MANUAL and AUTO modes during gameplay
            if (g_gameState == GameState::PLAYING) {
                g_manualObstacleMode = !g_manualObstacleMode;
                if (g_manualObstacleMode) {
                    std::cout << "[Switched to MANUAL mode - Click cells to toggle obstacles]" << std::endl;
                } else {
                    std::cout << "[Switched to AUTO mode - Obstacles toggle automatically]" << std::endl;
                }
                g_needsRender = true;
            }
            break;

        case 'T':
        case 't':
            // Toggle realtime comparator display and collection
            if (g_agent) {
                bool newVal = !g_agent->isComparatorEnabled();
                g_agent->setComparatorEnabled(newVal);
                std::cout << (newVal ? "[Realtime comparator ENABLED]" : "[Realtime comparator DISABLED]") << std::endl;
                g_needsRender = true;
            }
            break;
        
        case 'Q':
        case 'q':
        case VK_ESCAPE:
            // Quit or reset based on game state
            if (g_gameState == GameState::PLAYING || g_gameState == GameState::LEVEL_COMPLETE) {
                std::cout << "[ESC/Q pressed - Returning to main menu]" << std::endl;
                g_gameState = GameState::MAIN_MENU;
                g_needsRender = true;
            } else {
                PostQuitMessage(0);
            }
            break;
        }
        return 0;
    }
    }
    
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
