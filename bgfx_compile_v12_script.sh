#!/bin/bash
#
# Unix_vs_BGFX+++
# written by Andrea Giani
#
# Parameters:
#
# * no arguments 						: Normal compilation
# * --sse2 								: Enable SSE2
# * --avx2 								: Enable AVX2
# * --march=native 						: Specify architecture (native,haswell,skylake,etc)
# * --show-native-features				: Show defined Macros (useful for checking SSE/AVX/AVX2/etc.).
# * --run-examples						: Run BGFX examples after compilation
# * --run-starfield-demo				: Run custom starfield cube demo after compilation
# * --debug-starfield					: Enable debug mode for starfield demo  
# * --antivirus-safe					: Create antivirus-friendly version
# * --debug								: Debug build instead of Release
# * --shared							: Build shared libraries
#

set -e  # Terminate immediately on error
#set -x	# for debug

# Definitions
CPU_CORES=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Control variables
BUILD_TYPE="Release"
ENABLE_EXAMPLES=false
ENABLE_STARFIELD_DEMO=false
DEBUG_STARFIELD=false
ANTIVIRUS_SAFE=false
ENABLE_NASM=false
EXTRA_CXX_FLAGS=""
EXTRA_C_FLAGS=""
SHARED_LIBS="OFF"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
PURPLE='\033[0;35m'
NC='\033[0m'

export packages=(
  base-devel
  mingw-w64-x86_64-gcc
  mingw-w64-x86_64-cmake
  mingw-w64-x86_64-ninja
  mingw-w64-x86_64-glew
  mingw-w64-x86_64-glfw
  mingw-w64-x86_64-SDL2
  mingw-w64-x86_64-openal
  mingw-w64-x86_64-assimp
  mingw-w64-x86_64-stb
  mingw-w64-x86_64-boost
  mingw-w64-x86_64-bullet
  mingw-w64-x86_64-freealut
  mingw-w64-x86_64-freeimage
  mingw-w64-x86_64-pugixml
  mingw-w64-x86_64-glm
  mingw-w64-x86_64-glsl-optimizer
  mingw-w64-x86_64-sqlite3
  mingw-w64-x86_64-zziplib
  mingw-w64-x86_64-glib2
  mingw-w64-x86_64-pixman
  mingw-w64-x86_64-zlib 
  mingw-w64-x86_64-nasm
  bc
  git
  python3
  wget 
  curl
)
	
# Parsing arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --sse2)
            EXTRA_CXX_FLAGS+=" -msse2"
            EXTRA_C_FLAGS+=" -msse2"
            echo "SSE2 Enabled"
            shift
            ;;
        --avx2)
            EXTRA_CXX_FLAGS+=" -mavx2"
            EXTRA_C_FLAGS+=" -mavx2"
            echo "AVX2 Enabled"
            shift
            ;;
        --march=*)
            ARCH="${1#*=}"
            EXTRA_CXX_FLAGS+=" -march=$ARCH"
            EXTRA_C_FLAGS+=" -march=$ARCH"
            echo "Specified architecture: $ARCH"
            shift
            ;;
        --nasm)
            ENABLE_NASM=true
            echo "NASM Enabled"
            shift
            ;;			
        --run-examples)
            ENABLE_EXAMPLES=true
            shift
            ;;
        --run-starfield-demo)
            ENABLE_STARFIELD_DEMO=true
            shift
            ;;
        --debug-starfield)
            DEBUG_STARFIELD=true
            echo "Starfield debug mode enabled"
            shift
            ;;
        --antivirus-safe)
            ANTIVIRUS_SAFE=true
            echo "Antivirus-safe mode enabled"
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            echo "Debug build enabled"
            shift
            ;;
        --shared)
            SHARED_LIBS="ON"
            echo "Shared libraries enabled"
            shift
            ;;
        --show-native-features)
            echo "Macros enabled by -march=native:"
            gcc -march=native -dM -E - </dev/null | sort | grep '^#define __[[:upper:]]\+__\>'
            exit 0
            ;;                        
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Update and install the necessary packages
if [ -z "$(find /var/lib/pacman/sync -mtime -1 -print -quit 2>/dev/null)" ]; then
	echo -e "${BLUE}Update repository...${NC}"
	pacman -Sy --noconfirm || echo -e "${RED}Error on repository{NC}"
fi

echo -e "${BLUE}Installing/Updating required packages...${NC}"
for pkg in "${packages[@]}"; do
	if ! pacman -Q "$pkg" &> /dev/null; then
		echo "Installing required package $pkg..."
		pacman -S --needed --noconfirm "$pkg" || echo -e "${RED}Install Error on $pkg${NC}"
	fi
done

# Clone and prepare the BGFX repository
echo -e "${BLUE}Cloning BGFX repository...${NC}"
rm -rf ~/bgfx
rm -rf ~/bx
rm -rf ~/bimg
cd ~
git clone --recursive https://github.com/bkaradzic/bgfx.git
cd bgfx

# Also clone bx and bimg (BGFX dependencies)
cd ..
git clone https://github.com/bkaradzic/bx.git
git clone https://github.com/bkaradzic/bimg.git

# Return to bgfx directory
cd bgfx

# BGFX uses GENie build system, so we need to build it first
echo -e "${YELLOW}Building GENie build system...${NC}"
cd ../bx
make tools

# Return to bgfx and generate project files
cd ../bgfx

# Set up environment variables for BGFX build
export BX_DIR=$(realpath ../bx)
export BIMG_DIR=$(realpath ../bimg)

# Generate makefiles using GENie
echo -e "${YELLOW}Generating build files...${NC}"
../bx/tools/bin/windows/genie --with-examples --with-tools vs2022

# Build configuration with CMake alternative approach
echo -e "${YELLOW}Setting up CMake build...${NC}"
rm -rf build
mkdir -p build
cd build

# CMake configuration for BGFX
CMAKE_ARGS=(
    -G "Ninja"
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    -DCMAKE_INSTALL_PREFIX=/mingw64
    -DBGFX_BUILD_EXAMPLES=ON
    -DBGFX_BUILD_TOOLS=ON
    -DBUILD_SHARED_LIBS=$SHARED_LIBS
)

# Add extra optimization flags if specified
if [ -n "$EXTRA_CXX_FLAGS" ]; then
    CMAKE_ARGS+=(-DCMAKE_CXX_FLAGS="$EXTRA_CXX_FLAGS")
    CMAKE_ARGS+=(-DCMAKE_C_FLAGS="$EXTRA_C_FLAGS")

	if $ENABLE_NASM; then
		CMAKE_ARGS+=(-DCMAKE_ASM_NASM_FLAGS="-f win64 -Ox -Worphan-labels")
	fi
fi

# Try CMake build first, fallback to make if CMake fails
if [ -f ../CMakeLists.txt ]; then
	echo -e "${YELLOW}Using CMake build system...${NC}"
    cmake "${CMAKE_ARGS[@]}" ..
    ninja -j $CPU_CORES
else
	echo -e "${YELLOW}Using make build system...${NC}"

    cd ..
    
    # Set up make configuration
    MAKE_CONFIG="mingw64-gcc"
    if [ "$BUILD_TYPE" = "Debug" ]; then
        MAKE_CONFIG+="-debug"
    else
        MAKE_CONFIG+="-release"
    fi
    
    # Build using make
    make config=$MAKE_CONFIG -j$CPU_CORES
fi

echo -e "${GREEN}BGFX compiled successfully!${NC}"

# Install binaries to mingw64 (manual installation since BGFX doesn't have standard install)
echo -e "${YELLOW}Installing BGFX...${NC}"
mkdir -p /mingw64/include/bgfx
mkdir -p /mingw64/include/bx
mkdir -p /mingw64/include/bimg
mkdir -p /mingw64/lib
mkdir -p /mingw64/bin

# Copy BGFX headers
echo -e "${YELLOW}Installing BGFX headers...${NC}"
if [ -d "../include" ]; then
    cp -r ../include/* /mingw64/include/
fi

# Copy BX headers (dependency)
if [ -d "../../bx/include" ]; then
    cp -r ../../bx/include/* /mingw64/include/
fi

# Copy BIMG headers (dependency)  
if [ -d "../../bimg/include" ]; then
    cp -r ../../bimg/include/* /mingw64/include/
fi

# Find and copy libraries from build directories
echo -e "${YELLOW}Installing BGFX libraries...${NC}"
BUILD_DIRS=(".build" "build" "../.build")

for dir in "${BUILD_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo "Searching for libraries in $dir..."
        find "$dir" -name "*.a" -exec cp {} /mingw64/lib/ \; 2>/dev/null || true
        find "$dir" -name "*.dll" -exec cp {} /mingw64/bin/ \; 2>/dev/null || true
        find "$dir" -name "*.exe" -exec cp {} /mingw64/bin/ \; 2>/dev/null || true
    fi
done

# Also check parent directories for libraries
for dir in ../../bx ../../bimg; do
    if [ -d "$dir" ]; then
        find "$dir" -name "*.a" -exec cp {} /mingw64/lib/ \; 2>/dev/null || true
        find "$dir" -name "*.dll" -exec cp {} /mingw64/bin/ \; 2>/dev/null || true
    fi
done

# List what we actually installed
echo -e "${YELLOW}Installed headers:${NC}"
ls -la /mingw64/include/ | grep -E "(bgfx|bx|bimg)" || echo "No headers found"
echo -e "${YELLOW}Installed libraries:${NC}"
ls -la /mingw64/lib/ | grep -E "(bgfx|bx|bimg)" || echo "No libraries found"

# Create and compile the starfield demo
echo -e "${YELLOW}Creating starfield cube demo...${NC}"
mkdir -p ../starfield_demo
cd ../starfield_demo

cat > starfield_demo.cpp << 'EOF'
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include <math.h>

using namespace std;

// Check if we can find BGFX headers
#ifdef _WIN32
    #include <windows.h>
    #include <GL/gl.h>
    #pragma comment(lib, "opengl32.lib")
    #pragma comment(lib, "gdi32.lib")
    #pragma comment(lib, "user32.lib")
#endif

#include <GLFW/glfw3.h>

// Debug configuration - can be enabled via preprocessor
#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

// Debug macros
#if DEBUG_MODE
    #define DEBUG_PRINT(x) std::cout << "[DEBUG] " << x << std::endl
    #define DEBUG_CUBE(cube, id) debugCube(cube, id)
    #define DEBUG_GL_ERROR() checkGLError(__LINE__)
    #define DEBUG_STATS() printDebugStats()
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_CUBE(cube, id)
    #define DEBUG_GL_ERROR()
    #define DEBUG_STATS()
#endif

#ifndef _MATH_DEFINES_DEFINED 
	#define _MATH_DEFINES_DEFINED 

	#define M_E        2.71828182845904523536   // e 
	#define M_LOG2E    1.44269504088896340736   // log2(e) 
	#define M_LOG10E   0.434294481903251827651  // log10(e) 
	#define M_LN2      0.693147180559945309417  // ln(2) 
	#define M_LN10     2.30258509299404568402   // ln(10) 
	#define M_PI       3.14159265358979323846   // pi 
	#define M_PI_2     1.57079632679489661923   // pi/2 
	#define M_PI_4     0.785398163397448309616  // pi/4 
	#define M_1_PI     0.318309886183790671538  // 1/pi 
	#define M_2_PI     0.636619772367581343076  // 2/pi 
	#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi) 
	#define M_SQRT2    1.41421356237309504880   // sqrt(2) 
	#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2) 
#endif

// Global debug variables
int g_frameCount = 0;
double g_totalTime = 0.0;
int g_cubesReset = 0;
int g_cubesVisible = 0;
double g_minFrameTime = 999999.0;
double g_maxFrameTime = 0.0;

void checkGLError(int line) {
#if DEBUG_MODE
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cout << "[GL ERROR] Line " << line << ": ";
        switch(error) {
            case GL_INVALID_ENUM: std::cout << "GL_INVALID_ENUM"; break;
            case GL_INVALID_VALUE: std::cout << "GL_INVALID_VALUE"; break;
            case GL_INVALID_OPERATION: std::cout << "GL_INVALID_OPERATION"; break;
            case GL_OUT_OF_MEMORY: std::cout << "GL_OUT_OF_MEMORY"; break;
            default: std::cout << "Unknown error: " << error; break;
        }
        std::cout << std::endl;
    }
#endif
}

struct Cube {
    float x, y, z;
    float speed;
    float rotX, rotY, rotZ;
    float rotSpeedX, rotSpeedY, rotSpeedZ;
    float size;
    float r, g, b;
    int id;
    bool wasVisible;
    double creationTime;
    
    Cube(int cubeId = 0) : id(cubeId), wasVisible(false) {
        reset();
    }
    
    void reset() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-50.0f, 50.0f);
        std::uniform_real_distribution<> speed_dis(20.0f, 80.0f);
        std::uniform_real_distribution<> size_dis(0.3f, 1.5f);
        std::uniform_real_distribution<> rot_dis(-2.0f, 2.0f);
        std::uniform_real_distribution<> color_dis(0.3f, 1.0f);
        
        x = dis(gen);
        y = dis(gen);
        z = -300.0f - dis(gen);
        speed = speed_dis(gen);
        size = size_dis(gen);
        
        rotX = rotY = rotZ = 0.0f;
        rotSpeedX = rot_dis(gen);
        rotSpeedY = rot_dis(gen);
        rotSpeedZ = rot_dis(gen);
        
        r = color_dis(gen);
        g = color_dis(gen);
        b = color_dis(gen);
        
        wasVisible = false;
        creationTime = glfwGetTime();
        
        g_cubesReset++;
        
        DEBUG_PRINT("Cube " << id << " reset - Pos: (" << x << "," << y << "," << z << 
                   ") Speed: " << speed << " Size: " << size << 
                   " Color: (" << r << "," << g << "," << b << ")");
    }
    
    void update(float deltaTime) {
        float oldZ = z;
        z += speed * deltaTime;
        
        rotX += rotSpeedX * deltaTime;
        rotY += rotSpeedY * deltaTime;
        rotZ += rotSpeedZ * deltaTime;
        
        // Check visibility
        bool isVisible = (z > -300.0f && z < 10.0f);
        if (isVisible) g_cubesVisible++;
        
        if (!wasVisible && isVisible) {
            DEBUG_PRINT("Cube " << id << " became visible at Z=" << z);
        }
        wasVisible = isVisible;
        
        if (z > 10.0f) {
            DEBUG_PRINT("Cube " << id << " reached camera (Z=" << z << "), resetting after " << 
                       (glfwGetTime() - creationTime) << " seconds");
            reset();
        }
    }
    
    void draw() {
        glPushMatrix();
        
        DEBUG_GL_ERROR();
        
        glTranslatef(x, y, z);
        glRotatef(rotX * 57.2958f, 1.0f, 0.0f, 0.0f);
        glRotatef(rotY * 57.2958f, 0.0f, 1.0f, 0.0f);
        glRotatef(rotZ * 57.2958f, 0.0f, 0.0f, 1.0f);
        glScalef(size, size, size);
        
        glColor3f(r, g, b);
        
        DEBUG_GL_ERROR();
        
        // Draw cube using immediate mode
        glBegin(GL_QUADS);
        
        // Front face
        glVertex3f(-1.0f, -1.0f,  1.0f);
        glVertex3f( 1.0f, -1.0f,  1.0f);
        glVertex3f( 1.0f,  1.0f,  1.0f);
        glVertex3f(-1.0f,  1.0f,  1.0f);
        
        // Back face
        glVertex3f(-1.0f, -1.0f, -1.0f);
        glVertex3f(-1.0f,  1.0f, -1.0f);
        glVertex3f( 1.0f,  1.0f, -1.0f);
        glVertex3f( 1.0f, -1.0f, -1.0f);
        
        // Top face
        glVertex3f(-1.0f,  1.0f, -1.0f);
        glVertex3f(-1.0f,  1.0f,  1.0f);
        glVertex3f( 1.0f,  1.0f,  1.0f);
        glVertex3f( 1.0f,  1.0f, -1.0f);
        
        // Bottom face
        glVertex3f(-1.0f, -1.0f, -1.0f);
        glVertex3f( 1.0f, -1.0f, -1.0f);
        glVertex3f( 1.0f, -1.0f,  1.0f);
        glVertex3f(-1.0f, -1.0f,  1.0f);
        
        // Right face
        glVertex3f( 1.0f, -1.0f, -1.0f);
        glVertex3f( 1.0f,  1.0f, -1.0f);
        glVertex3f( 1.0f,  1.0f,  1.0f);
        glVertex3f( 1.0f, -1.0f,  1.0f);
        
        // Left face
        glVertex3f(-1.0f, -1.0f, -1.0f);
        glVertex3f(-1.0f, -1.0f,  1.0f);
        glVertex3f(-1.0f,  1.0f,  1.0f);
        glVertex3f(-1.0f,  1.0f, -1.0f);
        
        glEnd();
        
        DEBUG_GL_ERROR();
        
        glPopMatrix();
    }
};

void debugCube(const Cube& cube, int frameNum) {
#if DEBUG_MODE
    if (frameNum % 60 == 0 && cube.id < 5) { // Only debug first 5 cubes every 60 frames
        std::cout << "[CUBE " << cube.id << "] Pos: (" 
                  << std::fixed << std::setprecision(2) 
                  << cube.x << "," << cube.y << "," << cube.z << ") "
                  << "Rot: (" << cube.rotX << "," << cube.rotY << "," << cube.rotZ << ") "
                  << "Speed: " << cube.speed << " Size: " << cube.size << std::endl;
    }
#endif
}

void printDebugStats() {
#if DEBUG_MODE
    static int lastStatsFrame = 0;
    if (g_frameCount - lastStatsFrame >= 120) { // Every 2 seconds at 60fps
        std::cout << "\n=== DEBUG STATISTICS ===" << std::endl;
        std::cout << "Frame: " << g_frameCount << std::endl;
        std::cout << "Total Time: " << std::fixed << std::setprecision(2) << g_totalTime << "s" << std::endl;
        std::cout << "Cubes Reset: " << g_cubesReset << std::endl;
        std::cout << "Cubes Visible: " << g_cubesVisible << std::endl;
        std::cout << "Min Frame Time: " << std::fixed << std::setprecision(4) << g_minFrameTime * 1000.0 << "ms" << std::endl;
        std::cout << "Max Frame Time: " << std::fixed << std::setprecision(4) << g_maxFrameTime * 1000.0 << "ms" << std::endl;
        std::cout << "Avg FPS: " << std::fixed << std::setprecision(1) << g_frameCount / g_totalTime << std::endl;
        std::cout << "========================\n" << std::endl;
        lastStatsFrame = g_frameCount;
    }
#endif
}

class SmartSphere {
public:
    float x, y, z;
    float targetX, targetY, targetZ;
    float velocityX, velocityY, velocityZ;
    float rotationX, rotationY, rotationZ;
    float scale;
    float baseSize;
    float pulsationSpeed;
    float movementSpeed;
    float avoidanceRadius;
    float intelligenceLevel;
    float actionTimer;
    float actionDuration;
    float pulseIntensity;
    float glowIntensity;
    float blinkTimer;
    float blinkInterval;
    float maxGlow;
    bool blinkState;	
    int currentAction;
    bool debug;

    SmartSphere() 
        : x(0), y(0), z(-100),
          targetX(0), targetY(0), targetZ(-50),
          velocityX(0), velocityY(0), velocityZ(0),
          rotationX(0), rotationY(0), rotationZ(0),
          scale(1.0f), baseSize(3.0f),
          pulsationSpeed(1.5f),
          movementSpeed(8.0f),
          avoidanceRadius(12.0f),
          intelligenceLevel(0.5f),
          actionTimer(0.0f),
          actionDuration(3.0f),
          currentAction(0),
          pulseIntensity(0.0f),
          glowIntensity(1.0f),
          blinkTimer(0.0f),
          blinkInterval(0.8f),
          maxGlow(2.5f),
          blinkState(false),		  
          debug(false) 
    {
        chooseNewAction();
    }

    void chooseNewAction() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> action_dis(0, 6);
        std::uniform_real_distribution<> duration_dis(1.0f, 4.0f);
        
        currentAction = action_dis(gen);
        actionDuration = duration_dis(gen);
        actionTimer = 0.0f;
        
        // Calcola un nuovo target basato sull'azione corrente
        std::uniform_real_distribution<> pos_dis(-25.0f, 25.0f);
        std::uniform_real_distribution<> depth_dis(-150.0f, -50.0f);
        
        switch(currentAction) {
            case 0: // Movimento casuale
                targetX = pos_dis(gen);
                targetY = pos_dis(gen);
                targetZ = depth_dis(gen);
                break;
            case 1: // Movimento in avanti
                targetZ = z + 40.0f;
                if (targetZ > -30.0f) targetZ = -80.0f;
                break;
            case 2: // Movimento all'indietro
                targetZ = z - 40.0f;
                if (targetZ < -180.0f) targetZ = -120.0f;
                break;
            case 3: // Movimento laterale sinistro
                targetX = x - 20.0f;
                if (targetX < -35.0f) targetX = -25.0f;
                break;
            case 4: // Movimento laterale destro
                targetX = x + 20.0f;
                if (targetX > 35.0f) targetX = 25.0f;
                break;
            case 5: // Movimento verso l'alto
                targetY = y + 15.0f;
                if (targetY > 30.0f) targetY = 20.0f;
                break;
            case 6: // Movimento verso il basso
                targetY = y - 15.0f;
                if (targetY < -30.0f) targetY = -20.0f;
                break;
        }
        
        if (debug) {
            const char* actions[] = {"Random", "Forward", "Backward", "Left", "Right", "Up", "Down"};
            DEBUG_PRINT("Sphere chose action: " << actions[currentAction] 
                      << " | Target: (" << targetX << ", " << targetY << ", " << targetZ << ")");
        }
    }

    void avoidCubes(const std::vector<Cube>& cubes) {
        float avoidX = 0.0f;
        float avoidY = 0.0f;
        float avoidZ = 0.0f;
        int avoidCount = 0;

        for (const auto& cube : cubes) {
            float dx = cube.x - x;
            float dy = cube.y - y;
            float dz = cube.z - z;
            float dist = sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < avoidanceRadius) {
                float weight = 1.0f - (dist / avoidanceRadius);
                avoidX -= dx * weight;
                avoidY -= dy * weight;
                avoidZ -= dz * weight;
                avoidCount++;
                
                if (debug) {
                    DEBUG_PRINT("Sphere avoiding cube " << cube.id 
                              << " | Dist: " << dist 
                              << " | Weight: " << weight);
                }
            }
        }

        if (avoidCount > 0) {
            // Normalizza e applica le forze di evitamento
            float length = sqrt(avoidX*avoidX + avoidY*avoidY + avoidZ*avoidZ);
            if (length > 0) {
                avoidX /= length;
                avoidY /= length;
                avoidZ /= length;
                
                // Applica con intensità basata sul livello di intelligenza
                velocityX += avoidX * intelligenceLevel * 2.0f;
                velocityY += avoidY * intelligenceLevel * 2.0f;
                velocityZ += avoidZ * intelligenceLevel * 2.0f;
            }
        }
    }

    void update(float deltaTime, const std::vector<Cube>& cubes) {
        // Aggiorna il timer dell'azione
        actionTimer += deltaTime;
        if (actionTimer >= actionDuration) {
            chooseNewAction();
        }
        
        // Calcola la direzione verso il target
        float dirX = targetX - x;
        float dirY = targetY - y;
        float dirZ = targetZ - z;
        float distance = sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ);
        
        if (distance > 0.1f) {
            // Normalizza la direzione
            dirX /= distance;
            dirY /= distance;
            dirZ /= distance;
            
            // Applica la velocità verso il target
            velocityX += dirX * movementSpeed * deltaTime;
            velocityY += dirY * movementSpeed * deltaTime;
            velocityZ += dirZ * movementSpeed * deltaTime;
        }
        
        // Applica l'evitamento dei cubi
        avoidCubes(cubes);
        
        // Limita la velocità massima
        float speed = sqrt(velocityX*velocityX + velocityY*velocityY + velocityZ*velocityZ);
        if (speed > movementSpeed) {
            velocityX = (velocityX / speed) * movementSpeed;
            velocityY = (velocityY / speed) * movementSpeed;
            velocityZ = (velocityZ / speed) * movementSpeed;
        }
        
        // Applica attrito per un movimento più fluido
        velocityX *= 0.92f;
        velocityY *= 0.92f;
        velocityZ *= 0.92f;
        
        // Aggiorna la posizione
        x += velocityX * deltaTime;
        y += velocityY * deltaTime;
        z += velocityZ * deltaTime;
        
        // Aggiorna la rotazione basata sul movimento
        rotationX += velocityY * deltaTime * 0.5f;
        rotationY += velocityX * deltaTime * 0.5f;
        rotationZ += velocityZ * deltaTime * 0.3f;
        
        // Effetto di pulsazione
//      scale = baseSize * (0.8f + 0.2f * sin(glfwGetTime() * pulsationSpeed));
		
        // Aggiorna l'effetto di lampeggio
        blinkTimer += deltaTime;
        if (blinkTimer >= blinkInterval) {
            blinkState = !blinkState;
            blinkTimer = 0.0f;
            
            // Cambia casualmente l'intervallo di lampeggio
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> interval_dis(0.3f, 1.2f);
            blinkInterval = interval_dis(gen);
        }
        
        // Effetto di pulsazione più intenso
        pulseIntensity = 0.5f + 0.5f * sin(glfwGetTime() * pulsationSpeed * 2.0f);
        
        // Effetto di bagliore durante il lampeggio
        if (blinkState) {
            glowIntensity += deltaTime * 15.0f;
            if (glowIntensity > maxGlow) glowIntensity = maxGlow;
        } else {
            glowIntensity -= deltaTime * 8.0f;
            if (glowIntensity < 1.0f) glowIntensity = 1.0f;
        }
        
        // Combina pulsazione e bagliore
        scale = baseSize * (0.7f + 0.5f * pulseIntensity);		
    }

	void draw() {
		glPushMatrix();
		
		// Posizione e rotazione
		glTranslatef(x, y, z);
		glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
		glRotatef(rotationY, 0.0f, 1.0f, 0.0f);
		glRotatef(rotationZ, 0.0f, 0.0f, 1.0f);
		glScalef(scale, scale, scale);
		
		// Calcola il colore con effetto di bagliore
		float glowFactor = glowIntensity * (0.8f + 0.2f * pulseIntensity);
		float r = 0.2f * glowFactor;
		float g = 0.6f * glowFactor;
		float b = 1.0f * glowFactor;
		
		// Effetto "aura" esterna durante il lampeggio
		if (blinkState && glowIntensity > 1.5f) {
			// Disegna un'aura esterna
			glPushMatrix();
			glScalef(1.5f, 1.5f, 1.5f);
			glColor4f(r, g, b, 0.3f);
			
			// Sfera trasparente più grande
			const int auraSlices = 12;
			const int auraStacks = 12;
			for (int i = 0; i < auraStacks; ++i) {
				float phi1 = static_cast<float>(i) / auraStacks * M_PI;
				float phi2 = static_cast<float>(i+1) / auraStacks * M_PI;
				
				glBegin(GL_QUAD_STRIP);
				for (int j = 0; j <= auraSlices; ++j) {
					float theta = static_cast<float>(j) / auraSlices * 2.0f * M_PI;
					
					for (int k = 0; k < 2; ++k) {
						float phi = (k == 0) ? phi1 : phi2;
						float x = sin(phi) * cos(theta);
						float y = sin(phi) * sin(theta);
						float z = cos(phi);
						glVertex3f(x, y, z);
					}
				}
				glEnd();
			}
			glPopMatrix();
		}
		
		// Corpo principale della sfera
		glColor3f(r, g, b);
		
		// Disegna la sfera principale
		const int slices = 16;
		const int stacks = 16;
		for (int i = 0; i < stacks; ++i) {
			float phi1 = static_cast<float>(i) / stacks * M_PI;
			float phi2 = static_cast<float>(i+1) / stacks * M_PI;
			
			glBegin(GL_QUAD_STRIP);
			for (int j = 0; j <= slices; ++j) {
				float theta = static_cast<float>(j) / slices * 2.0f * M_PI;
				
				for (int k = 0; k < 2; ++k) {
					float phi = (k == 0) ? phi1 : phi2;
					float x = sin(phi) * cos(theta);
					float y = sin(phi) * sin(theta);
					float z = cos(phi);
					
					// Illuminazione dinamica
					float light = (z + 1.0f) * 0.4f + 0.2f;
					float currentR = r * light;
					float currentG = g * light;
					float currentB = b * light;
					glColor3f(currentR, currentG, currentB);
					
					glVertex3f(x, y, z);
				}
			}
			glEnd();
		}
		
		// Nucleo interno pulsante
		glPushMatrix();
		float coreSize = 0.4f + 0.1f * pulseIntensity;
		glScalef(coreSize, coreSize, coreSize);
		
		// Cambia colore durante il lampeggio
		if (blinkState) {
			// Bianco brillante con effetto energetico
			float coreGlow = glowIntensity * 1.5f;
			if (coreGlow > 3.0f) coreGlow = 3.0f;
			glColor3f(1.0f, 1.0f, 0.7f * coreGlow);
		} else {
			// Giallo pulsante
			float yellowIntensity = 0.8f + 0.2f * sin(glfwGetTime() * pulsationSpeed * 3.0f);
			glColor3f(1.0f, 0.8f, 0.2f * yellowIntensity);
		}
		
		// Nucleo più dettagliato
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= slices; ++j) {
			float theta = static_cast<float>(j) / slices * 2.0f * M_PI;
			float x = cos(theta);
			float y = sin(theta);
			
			// Effetto a strisce per il nucleo
			if (j % 4 == 0) {
				glColor3f(1.0f, 1.0f, 0.9f);
			} else {
				if (blinkState) {
					glColor3f(1.0f, 1.0f, 0.7f);
				} else {
					glColor3f(1.0f, 0.9f, 0.4f);
				}
			}
			
			glVertex3f(x, y, 0.0f);
			glVertex3f(x * 0.8f, y * 0.8f, 0.5f);
		}
		glEnd();
		
		glPopMatrix();
		
		// Effetto di scia durante il movimento
		if ((velocityX != 0 || velocityY != 0 || velocityZ != 0) && blinkState) {
			glPushMatrix();
			glTranslatef(-velocityX * 0.3f, -velocityY * 0.3f, -velocityZ * 0.3f);
			glScalef(0.7f, 0.7f, 0.7f);
			glColor4f(r, g, b, 0.4f);
			
			// Sfera di scia più semplice
			const int trailSlices = 10;
			const int trailStacks = 10;
			for (int i = 0; i < trailStacks; ++i) {
				float phi1 = static_cast<float>(i) / trailStacks * M_PI;
				float phi2 = static_cast<float>(i+1) / trailStacks * M_PI;
				
				glBegin(GL_QUAD_STRIP);
				for (int j = 0; j <= trailSlices; ++j) {
					float theta = static_cast<float>(j) / trailSlices * 2.0f * M_PI;
					
					for (int k = 0; k < 2; ++k) {
						float phi = (k == 0) ? phi1 : phi2;
						float x = sin(phi) * cos(theta);
						float y = sin(phi) * sin(theta);
						float z = cos(phi);
						glVertex3f(x, y, z);
					}
				}
				glEnd();
			}
			glPopMatrix();
		}
		
		// Effetto particellare durante il lampeggio
		if (blinkState && glowIntensity > 2.0f) {
			glPushMatrix();
			glScalef(1.2f, 1.2f, 1.2f);
			glColor4f(0.8f, 0.9f, 1.0f, 0.6f);
			
			// Particelle direzionali
			const int particles = 12;
			glBegin(GL_LINES);
			for (int i = 0; i < particles; i++) {
				float angle = 2.0f * M_PI * i / particles;
				float cx = cos(angle);
				float cy = sin(angle);
				
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(cx * 2.5f, cy * 2.5f, 0.0f);
			}
			glEnd();
			glPopMatrix();
		}
		
		glPopMatrix();
	}

};

class Spaceship {
public:
    float x, y, z;
    float velocityX, velocityY;
    float avoidanceRadius;
    float maxSpeed;
    float avoidanceForce;
    float steeringSpeed;
    int id;
    bool debug;

    Spaceship() 
        : x(0), y(0), z(-10), 
          velocityX(0), velocityY(0),
          avoidanceRadius(8.0f), maxSpeed(15.0f),
          avoidanceForce(40.0f), steeringSpeed(0.5f),
          id(-1), debug(false) {}

    void update(float deltaTime, const std::vector<Cube>& cubes) {
        // Calcola forze di evitamento
        float forceX = 0.0f;
        float forceY = 0.0f;
        int threats = 0;

        for (const auto& cube : cubes) {
            if (cube.z > z - 30.0f && cube.z < z + 10.0f) {
                float dx = cube.x - x;
                float dy = cube.y - y;
                float distance = std::sqrt(dx*dx + dy*dy);

                if (distance < avoidanceRadius) {
                    float weight = 1.0f - (distance / avoidanceRadius);
                    forceX -= dx * weight;
                    forceY -= dy * weight;
                    threats++;
                    
                    if (debug) {
                        DEBUG_PRINT("🚨 Spaceship avoiding cube " << cube.id 
                            << " at distance: " << distance);
                    }
                }
            }
        }

        // Normalizza e applica le forze
        if (threats > 0) {
            float magnitude = std::sqrt(forceX*forceX + forceY*forceY);
            if (magnitude > 0) {
                forceX = (forceX / magnitude) * avoidanceForce;
                forceY = (forceY / magnitude) * avoidanceForce;
            }
        }

        // Movimento casuale quando non ci sono minacce
        if (threats == 0) {
            forceX += (rand() % 100 - 50) * 0.01f;
            forceY += (rand() % 100 - 50) * 0.01f;
        }

        // Aggiorna velocità
        velocityX += forceX * deltaTime;
        velocityY += forceY * deltaTime;

        // Limita velocità
        float speed = std::sqrt(velocityX*velocityX + velocityY*velocityY);
        if (speed > maxSpeed) {
            velocityX = (velocityX / speed) * maxSpeed;
            velocityY = (velocityY / speed) * maxSpeed;
        }

        // Aggiorna posizione
        x += velocityX * deltaTime;
        y += velocityY * deltaTime;

        // Mantieni entro i bordi
        const float boundary = 25.0f;
        if (x < -boundary) x = -boundary;
        if (x > boundary) x = boundary;
        if (y < -boundary) y = -boundary;
        if (y > boundary) y = boundary;

        // Effetto di inerzia
        velocityX *= 0.95f;
        velocityY *= 0.95f;
    }

    void draw() {
        glPushMatrix();
        glTranslatef(x, y, z);
        glRotatef(std::atan2(velocityY, velocityX) * 57.2958f, 0, 0, 1);
        
        // Corpo navicella
        glColor3f(0.2f, 0.8f, 1.0f);
        glBegin(GL_TRIANGLES);
            glVertex3f(1.0f, 0.0f, 0.0f);
            glVertex3f(-0.8f, 0.6f, 0.0f);
            glVertex3f(-0.8f, -0.6f, 0.0f);
        glEnd();
        
        // Motori
        glColor3f(1.0f, 0.2f, 0.1f);
        glBegin(GL_QUADS);
            glVertex3f(-1.0f, 0.4f, 0.0f);
            glVertex3f(-1.4f, 0.4f, 0.0f);
            glVertex3f(-1.4f, -0.4f, 0.0f);
            glVertex3f(-1.0f, -0.4f, 0.0f);
        glEnd();
        
        glPopMatrix();
    }
};

const int MAX_CUBES = 150;
std::vector<Cube> cubes;

void setupGL() {
    DEBUG_PRINT("Setting up OpenGL...");
    
    glEnable(GL_DEPTH_TEST);
    DEBUG_GL_ERROR();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    DEBUG_GL_ERROR();
    
    glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
    DEBUG_GL_ERROR();
    
    // Setup perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    DEBUG_GL_ERROR();
    
    // gluPerspective equivalent
    float fov = 60.0f;
    float aspect = 1920.0f / 1080.0f;
    float zNear = 0.1f;
    float zFar = 1000.0f;
    
    float fH = tan(fov / 360.0f * 3.14159f) * zNear;
    float fW = fH * aspect;
    glFrustum(-fW, fW, -fH, fH, zNear, zFar);
    DEBUG_GL_ERROR();
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    DEBUG_GL_ERROR();
    
    DEBUG_PRINT("OpenGL setup complete");
    
    // Print OpenGL info
    DEBUG_PRINT("OpenGL Version: " << glGetString(GL_VERSION));
    DEBUG_PRINT("OpenGL Vendor: " << glGetString(GL_VENDOR));
    DEBUG_PRINT("OpenGL Renderer: " << glGetString(GL_RENDERER));
}

void handleKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        switch(key) {
            case GLFW_KEY_ESCAPE:
                DEBUG_PRINT("ESC pressed - closing window");
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_D:
                std::cout << "\n=== INSTANT DEBUG INFO ===" << std::endl;
                std::cout << "Current Frame: " << g_frameCount << std::endl;
                std::cout << "Visible Cubes: " << g_cubesVisible << std::endl;
                std::cout << "Total Resets: " << g_cubesReset << std::endl;
                std::cout << "Runtime: " << g_totalTime << "s" << std::endl;
                std::cout << "=========================\n" << std::endl;
                break;
            case GLFW_KEY_R:
                DEBUG_PRINT("R pressed - resetting all cubes");
                g_cubesReset = 0;
                for (auto& cube : cubes) {
                    cube.reset();
                }
                break;
            case GLFW_KEY_SPACE:
                DEBUG_PRINT("SPACE pressed - pausing for 2 seconds");
                glfwWaitEventsTimeout(2.0);
                break;
        }
    }
}

int main() {
    std::cout << ">BGFX Starfield Cube Demo (OpenGL Fallback)<" << std::endl;
    std::cout << "============================================" << std::endl;
    
#if DEBUG_MODE
    std::cout << ">>DEBUG MODE ENABLED<<" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  [D]		- Show debug info" << std::endl;
    std::cout << "  [R]		- Reset all cubes" << std::endl;
    std::cout << "  [SPACE] - Pause for 2 seconds" << std::endl;
    std::cout << "  [ESC]	- Exit" << std::endl;
    std::cout << "============================================" << std::endl;
#endif
    
    // Initialize cube array
    cubes.reserve(MAX_CUBES);
    for (int i = 0; i < MAX_CUBES; i++) {
        cubes.emplace_back(i);
    }
    
    DEBUG_PRINT("Initialized " << MAX_CUBES << " cubes");

    // Initialize spaceship
    Spaceship spaceship;
    spaceship.id = 999;
    if (DEBUG_MODE) {
        spaceship.debug = true;
        std::cout << "Spaceship initialized at (0,0,-10)" << std::endl;
    }

    // Initialize AI Sphere
    SmartSphere aiSphere;
    if (DEBUG_MODE) {
        aiSphere.debug = true;
        std::cout << "AI Sphere initialized" << std::endl;
    }
    
    // Initialize GLFW
    DEBUG_PRINT("Initializing GLFW...");
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "BGFX Starfield Cube Demo - OpenGL Fallback", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    DEBUG_PRINT("GLFW window created successfully");
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // VSync
    glfwSetKeyCallback(window, handleKeyboard);
    
    setupGL();
    
    std::cout << "Demo started! Close window to exit." << std::endl;
    std::cout << "Rendering " << MAX_CUBES << " animated cubes..." << std::endl;
    
    double lastTime = glfwGetTime();
    int frameCount = 0;
    double fpsTimer = 0.0;
    
    DEBUG_PRINT("Entering main loop...");
    
    while (!glfwWindowShouldClose(window)) {
        double currentTime = glfwGetTime();
        float deltaTime = float(currentTime - lastTime);
        lastTime = currentTime;
        
        g_frameCount++;
        g_totalTime = currentTime;
        g_cubesVisible = 0;
        
        // Track frame time statistics
        if (deltaTime < g_minFrameTime) g_minFrameTime = deltaTime;
        if (deltaTime > g_maxFrameTime) g_maxFrameTime = deltaTime;
        
        // FPS counter
        frameCount++;
        fpsTimer += deltaTime;
        if (fpsTimer >= 1.0) {
            std::cout << "FPS: " << frameCount << " | Cubes: " << MAX_CUBES 
                      << " | Visible: " << g_cubesVisible 
                      << " | Resets: " << g_cubesReset 
                      << " | Frame: " << g_frameCount << std::endl;
            frameCount = 0;
            fpsTimer = 0.0;
        }
        
        // Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        DEBUG_GL_ERROR();
        
        // Reset modelview matrix
        glLoadIdentity();
        DEBUG_GL_ERROR();
        
        // Update and render cubes
        for (auto& cube : cubes) {
            cube.update(deltaTime);
            DEBUG_CUBE(cube, g_frameCount);
            cube.draw();
        }

        // Update and render spaceship
    //  spaceship.update(deltaTime, cubes);
    //  spaceship.draw();
 
        // Update and render the sphere
        aiSphere.update(deltaTime, cubes);
        aiSphere.draw();
		
        DEBUG_STATS();
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    DEBUG_PRINT("Exiting main loop...");
    
    std::cout << "\n=== FINAL STATISTICS ===" << std::endl;
    std::cout << "Total Frames: " << g_frameCount << std::endl;
    std::cout << "Total Runtime: " << std::fixed << std::setprecision(2) << g_totalTime << " seconds" << std::endl;
    std::cout << "Average FPS: " << std::fixed << std::setprecision(1) << g_frameCount / g_totalTime << std::endl;
    std::cout << "Total Cube Resets: " << g_cubesReset << std::endl;
    std::cout << "Min Frame Time: " << std::fixed << std::setprecision(4) << g_minFrameTime * 1000.0 << "ms" << std::endl;
    std::cout << "Max Frame Time: " << std::fixed << std::setprecision(4) << g_maxFrameTime * 1000.0 << "ms" << std::endl;
    std::cout << "========================" << std::endl;
    
    glfwDestroyWindow(window);
    glfwTerminate();
    
    std::cout << "Demo finished. Thank you!" << std::endl;
    return 0;
}
EOF

# Create a simple Makefile for the demo
# Create a Makefile with debug options
cat > Makefile << 'EOF'
CXX = g++
CXXFLAGS = -std=c++11 -I/mingw64/include
LDFLAGS = -L/mingw64/lib
LIBS = -lglfw3 -lopengl32 -lgdi32 -luser32 -lkernel32

TARGET = starfield_demo
SOURCES = starfield_demo.cpp

# Default build (optimized)
$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -O2 -DDEBUG_MODE=0 $(SOURCES) $(LDFLAGS) $(LIBS) -o $(TARGET)

# Debug build
debug: $(SOURCES)
	$(CXX) $(CXXFLAGS) -g -O0 -DDEBUG_MODE=1 -DDEBUG $(SOURCES) $(LDFLAGS) $(LIBS) -o $(TARGET)_debug

# Antivirus-safe build (static linking, different name)
safe: $(SOURCES)
	$(CXX) $(CXXFLAGS) -O2 -DDEBUG_MODE=0 -static -static-libgcc -static-libstdc++ $(SOURCES) $(LDFLAGS) $(LIBS) -o cube_animation

# Verbose debug build
debug-verbose: $(SOURCES)
	$(CXX) $(CXXFLAGS) -g -O0 -DDEBUG_MODE=1 -DDEBUG -v $(SOURCES) $(LDFLAGS) $(LIBS) -o $(TARGET)_debug

clean:
	rm -f $(TARGET) $(TARGET).exe $(TARGET)_debug $(TARGET)_debug.exe cube_animation cube_animation.exe

.PHONY: clean debug debug-verbose safe
EOF

echo -e "${YELLOW}Compiling starfield demo...${NC}"

# Choose compilation mode based on flags
if $ANTIVIRUS_SAFE; then
    echo -e "${YELLOW}Building ANTIVIRUS-SAFE version (static linking)...${NC}"
    if make safe; then
        echo "Antivirus-safe demo compiled successfully as 'cube_animation'!"
        if [ -f cube_animation.exe ]; then
            cp cube_animation.exe /mingw64/bin/
            echo "Safe demo installed to /mingw64/bin/cube_animation.exe"
        elif [ -f cube_animation ]; then
            cp cube_animation /mingw64/bin/
            echo "Safe demo installed to /mingw64/bin/cube_animation"
        fi
    else
        echo -e "${RED}Warning: Antivirus-safe demo compilation failed.${NC}"
    fi
elif $DEBUG_STARFIELD; then
    echo -e "${YELLOW}Building in DEBUG mode...${NC}"
    if make debug; then
        echo -e "${GREEN}Starfield DEBUG demo compiled successfully!${NC}"
        if [ -f starfield_demo_debug.exe ]; then
            cp starfield_demo_debug.exe /mingw64/bin/
            echo "DEBUG demo installed to /mingw64/bin/starfield_demo_debug.exe"
        elif [ -f starfield_demo_debug ]; then
            cp starfield_demo_debug /mingw64/bin/
            echo "DEBUG demo installed to /mingw64/bin/starfield_demo_debug"
        fi
    else
        echo "Warning: Starfield DEBUG demo compilation failed."
    fi
else
	echo -e "${YELLOW}Building in RELEASE mode...${NC}"
    if make; then
        echo -e "${GREEN}Starfield demo compiled successfully!${NC}"
        if [ -f starfield_demo.exe ]; then
            cp starfield_demo.exe /mingw64/bin/
            echo "Demo installed to /mingw64/bin/starfield_demo.exe"
        elif [ -f starfield_demo ]; then
            cp starfield_demo /mingw64/bin/
            echo "Demo installed to /mingw64/bin/starfield_demo"
        fi
    else
        echo "Warning: Starfield demo compilation failed."
    fi
fi

# Create launcher script for antivirus issues
cat > run_demo.bat << 'EOF'
@echo off
echo Starting Cube Animation Demo...
echo If Windows Defender asks, click "More info" then "Run anyway"
echo Press Any Key
pause
PATH ..\..\..\..\msys64\mingw64\bin\

IF EXIST "./cube_animation.exe" (
    cube_animation.exe
) ELSE (
    cube_animation_debug.exe
)

IF EXIST "./starfield_demo.exe" (
    starfield_demo.exe
) ELSE (
    starfield_demo_debug.exe
)

pause
EOF

echo -e "${GREEN}Created run_demo.bat launcher for Windows compatibility${NC}"

cd ../bgfx

# Run examples if requested
if $ENABLE_EXAMPLES; then
    echo "Looking for BGFX examples..."
    
    # Find example executables
    EXAMPLE_DIRS=(.build build)
    FOUND_EXAMPLES=false
    
    for dir in "${EXAMPLE_DIRS[@]}"; do
        if [ -d "$dir" ]; then
            EXAMPLES=$(find "$dir" -name "*example*" -type f -executable 2>/dev/null || true)
            if [ -n "$EXAMPLES" ]; then
                FOUND_EXAMPLES=true
                echo "Found BGFX examples:"
                echo "$EXAMPLES"
                
                # Run the first example found
                FIRST_EXAMPLE=$(echo "$EXAMPLES" | head -n1)
                if [ -f "$FIRST_EXAMPLE" ]; then
                    echo "Starting BGFX example: $FIRST_EXAMPLE"
                    "$FIRST_EXAMPLE" &
                fi
                break
            fi
        fi
    done
    
    if [ "$FOUND_EXAMPLES" = false ]; then
        echo -e "${RED}Warning: No BGFX examples found. The build may have failed or examples weren't built.${NC}"
    fi
fi

# Run starfield demo if requested
if $ENABLE_STARFIELD_DEMO; then
    echo "Starting BGFX Starfield Cube Demo..."
    
    if $ANTIVIRUS_SAFE; then
        echo " Running ANTIVIRUS-SAFE version..."
        if [ -f "/mingw64/bin/cube_animation.exe" ]; then
            echo "Running safe cube animation..."
            echo " If Antivirus blocks it, add /mingw64/bin/ to exceptions!"
            /mingw64/bin/cube_animation.exe &
        elif [ -f "/mingw64/bin/cube_animation" ]; then
            echo "Running safe cube animation..."
            /mingw64/bin/cube_animation &
        elif [ -f "../starfield_demo/cube_animation.exe" ]; then
            echo "Running safe cube animation from build directory..."
            ../starfield_demo/cube_animation.exe &
        elif [ -f "../starfield_demo/cube_animation" ]; then
            echo "Running safe cube animation from build directory..."
            ../starfield_demo/cube_animation &
        else
            echo -e "${RED}Warning: Safe cube animation not found. The build may have failed.${NC}"
        fi
    elif $DEBUG_STARFIELD; then
        echo " Running DEBUG version with detailed logging..."
        if [ -f "/mingw64/bin/starfield_demo_debug.exe" ]; then
            echo "Running DEBUG starfield demo..."
            /mingw64/bin/starfield_demo_debug.exe &
        elif [ -f "/mingw64/bin/starfield_demo_debug" ]; then
            echo "Running DEBUG starfield demo..."
            /mingw64/bin/starfield_demo_debug &
        elif [ -f "../starfield_demo/starfield_demo_debug.exe" ]; then
            echo "Running DEBUG starfield demo from build directory..."
            ../starfield_demo/starfield_demo_debug.exe &
        elif [ -f "../starfield_demo/starfield_demo_debug" ]; then
            echo "Running DEBUG starfield demo from build directory..."
            ../starfield_demo/starfield_demo_debug &
        else
            echo -e "${RED}Warning: DEBUG Starfield demo not found. The build may have failed.${NC}"
        fi
    else
        if [ -f "/mingw64/bin/starfield_demo.exe" ]; then
            echo "Running starfield demo..."
            echo "  If your antivirus blocks it, try --antivirus-safe option"
            /mingw64/bin/starfield_demo.exe &
        elif [ -f "/mingw64/bin/starfield_demo" ]; then
            echo "Running starfield demo..."
            /mingw64/bin/starfield_demo &
        elif [ -f "../starfield_demo/starfield_demo.exe" ]; then
            echo "Running starfield demo from build directory..."
            ../starfield_demo/starfield_demo.exe &
        elif [ -f "../starfield_demo/starfield_demo" ]; then
            echo "Running starfield demo from build directory..."
            ../starfield_demo/starfield_demo &
        else
            echo -e "${RED}Warning: Starfield demo not found. The build may have failed.${NC}"
        fi
    fi
fi

echo ""
echo "==================================="
echo -e "${GREEN}BGFX BUILD COMPLETED SUCCESSFULLY!${NC}"
echo "==================================="
echo -e "${GREEN}BGFX Test by Andrea Giani${NC}"
echo ""
echo -e "${PURPLE}Libraries installed to: /mingw64/lib${NC}"
echo -e "${PURPLE}Headers installed to:   /mingw64/include${NC}"
echo -e "${PURPLE}Binaries installed to:  /mingw64/bin${NC}"
echo ""
echo -e "${YELLOW}To use BGFX in your projects, link with:${NC}"
echo "  -lbgfx -lbimg -lbx"
echo ""
echo -e "${YELLOW}Example compile command:${NC}"
echo "  g++ -I/mingw64/include your_app.cpp -L/mingw64/lib -lbgfx -lbimg -lbx -lopengl32 -lgdi32"
echo ""
echo "Custom starfield demo available at: /mingw64/bin/starfield_demo.exe"
if $DEBUG_STARFIELD || [ -f "/mingw64/bin/starfield_demo_debug.exe" ]; then
    echo "Debug version available at: /mingw64/bin/starfield_demo_debug.exe"
fi
if $ANTIVIRUS_SAFE || [ -f "/mingw64/bin/cube_animation.exe" ]; then
    echo "Antivirus-safe version at: /mingw64/bin/cube_animation.exe"
fi
echo ""
echo -e "${YELLOW}Usage examples:${NC}"
echo "  ./this script.sh --run-starfield-demo                   # Run normal demo"
echo "  ./this script.sh --antivirus-safe --run-starfield-demo  # Antivirus-friendly version"
echo "  ./this script.sh --debug-starfield --run-starfield-demo # Run with debug info"
echo ""
echo -e "${YELLOW}For Antivirus issues:${NC}"
echo "  1. Add /mingw64/bin/ to Antivirus exceptions"
echo "  2. Use --antivirus-safe option for static build"
echo "  3. Check quarantine for blocked files"
echo ""
echo -e "${YELLOW}Debug demo controls:${NC}"
echo "  [D]			- Show instant debug statistics"
echo "  [R]			- Reset all cubes"
echo "  [SPACE]		- Pause for 2 seconds"
echo "  [ESC]			- Exit demo"
echo ""
