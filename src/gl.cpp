#include "glad/glad.h"  // glad必须在GLFW/GL之前include
#include "GL/gl.h"
#include "GLFW/glfw3.h"

#include <time.h>

#include <iostream>
#include <list>
#include <mutex>
#include <thread>

#include "BlockingQueue.h"
#include "Sphere.h"
#include "camera.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glog/logging.h"
#include "shader_m.h"
// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"

// FFMPEG
extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixfmt.h"
#include "libavutil/time.h"
#include "libswscale/swscale.h"
}

using namespace std;
using namespace google;

const float PI = acos(-1);

void startRendor();
int rendorThreadFunc();

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action,
                           int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);
void key_callback(GLFWwindow* window, int key, int scancode, int action,
                  int mods);

// queue
extern BlockingQueue<AVFrame*> picQue;
// thread
thread* rendorThread;

// texture
unsigned int texture1;
int texW = 0;
int texH = 0;

// settings
const unsigned int SCR_WIDTH = 640;
const unsigned int SCR_HEIGHT = 360;
int viewMode;  // sphere/rectangle

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 0.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
float zoomSpeed = 32.0f;

// predict
bool displayPredict = true;
float predictScale = 1.2f;  // 预测视角缩放
float predictExtra = 24;    // 额外预测视角
Camera predictCamera(glm::vec3(0.0f, 0.0f, 0.0f));
vector<glm::vec2> visibleVertices;
vector<glm::vec2> predictedVisibleVertices;

// FoV
bool frozeFoV = false;
list<glm::vec3> focusTrack;
int focusTexS, focusTexT;
float focusS, focusT;
bool displayFoV = true;
float extraFoV = 0.0f;  // 扩展显示FoV

// sphere
const int numSectors = 144;
const int numStacks = 72;
Sphere sphere(1.0f, numSectors,
              numStacks);  // radius, sectors, stacks, non-smooth (flat) shading

// timing
float deltaTime = 0.0f;  // time between current frame and last frame
float lastFrame = 0.0f;

void startRendor() {
  cout << "rendor thread started." << endl;
  rendorThread = new thread(rendorThreadFunc);
}

template <typename T>
void LeastSquare(const vector<T>& lx, const vector<T>& ly, const int n,
                 double& a, double& b) {
  T t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  auto itx = lx.begin();
  auto ity = ly.begin();
  double x, y;
  for (int i = 0; i < n; ++i) {
    x = *itx++, y = *ity++;
    t1 += x * x;
    t2 += x;
    t3 += x * y;
    t4 += y;
  }
  a = (double)(t3 * n - t2 * t4) / (double)(t1 * n - t2 * t2);   // 求得β1
  b = (double)(t1 * t4 - t2 * t3) / (double)(t1 * n - t2 * t2);  // 求得β2
}

template <typename T>
double predict(const vector<T>& lx, const vector<T>& ly, const int n,
               const double& x) {
  double a, b;
  LeastSquare(lx, ly, n, a, b);
  return a * x + b;
}

int genView(unsigned int& VBO, unsigned int& VAO, unsigned int& EBO,
            unsigned int& EBO_lines, const float* vertices,
            const unsigned int* indices, const unsigned int* lineIndices,
            unsigned size, unsigned indexSize, unsigned lineIndexSize,
            unsigned stride) {
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);
  glGenBuffers(1, &EBO_lines);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexSize, indices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_lines);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, lineIndexSize, lineIndices,
               GL_STATIC_DRAW);

  // position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
  glEnableVertexAttribArray(0);
  // texture coord attribute
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride,
                        (void*)(6 * sizeof(float)));
  glEnableVertexAttribArray(1);

  return 0;
}

void calculateFoV() {
  if (frozeFoV) return;

  static std::mutex mtx;
  mtx.lock();

  // get focus and FoV
  // sphere
  const float* verticesSphere = sphere.getInterleavedVertices();
  unsigned strideSphere = sphere.getInterleavedStride();
  {
    // get focus in texture
    float yaw, pitch;
    yaw = camera.Yaw, pitch = camera.Pitch;
    const float PI = acos(-1);
    float sectorAngle = glm::radians(yaw);
    float stackAngle = glm::radians(pitch);
    focusS = sectorAngle / (PI * 2);
    focusT = 0.5f - stackAngle / PI;

    // predict focus s,t
    double predictedYaw, predictedPitch;
    unsigned int predictWindow = 16;  // FIXME: by config
    double predicteDuration = 500.0;  // FIXME: by config
    double now = (double)clock() / 1000.0;
    focusTrack.push_back(glm::vec3(yaw, pitch, now));

    predictedYaw = camera.getYaw();
    predictedPitch = camera.getPitch();
    if (focusTrack.size() > predictWindow) {
      focusTrack.pop_front();
      float yaw, pitch, clk;
      vector<float> lYaw, yPitch, lclk;
      for (auto& record : focusTrack) {
        yaw = record.x, pitch = record.y, clk = record.z;
        lYaw.push_back(yaw);
        yPitch.push_back(pitch);
        lclk.push_back(clk);
      }
      predictedYaw = predict(lclk, lYaw, predictWindow, now + predicteDuration);
      predictedPitch =
          predict(lclk, yPitch, predictWindow, now + predicteDuration);
      VLOG(INFO) << "now " << now << " predict " << yaw << "," << pitch
                 << " -> " << predictedYaw << "," << predictedPitch << endl;
    }

    predictCamera = camera;
    predictCamera.Zoom =
        camera.Zoom * predictScale + predictExtra;  // FIXME: by config
    // auto diff = glm::vec2(focusTrack.back() - focusTrack.front());//hack
    // case if(glm::length(diff)<5){
    //   predictCamera.Zoom*=1.5f;
    // }

    predictCamera.setYaw(predictedYaw);
    predictCamera.setPitch(predictedPitch);
    // predictCamera.ProcessMouseMovement(predictedYaw - camera.Yaw,
    //                                    predictedPitch - camera.Pitch);
    predictCamera.updateCameraVectors();
    // get FoV in texture
    {
      glm::mat4 projection(1.0), view(1.0), model(1.0), mvp(1.0);
      glm::mat4 projectionPredict(1.0), viewPredict(1.0), mvpPredict(1.0);
      model =
          glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
      // camera/view transformation
      view = camera.GetViewMatrix();
      projection =
          glm::perspective(glm::radians(camera.Zoom),
                           (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
      mvp = projection * view * model;

      viewPredict = predictCamera.GetViewMatrix();
      projectionPredict =
          glm::perspective(glm::radians(predictCamera.Zoom),
                           (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
      mvpPredict = projectionPredict * viewPredict * model;
      auto visible = [](glm::vec4& v, float limit = 1.0f) {
        v /= v.w;
        return v.x >= -limit && v.x <= limit && v.y >= -limit && v.y <= limit &&
               v.z >= -limit && v.z <= limit;
      };

      // vertices in FoV
      visibleVertices.clear();
      predictedVisibleVertices.clear();
      for (int i = 0; i < numStacks; i++) {
        for (int j = 0; j < numSectors; j++) {
          auto vertex = verticesSphere +
                        (i * numSectors + j) * strideSphere / sizeof(float);
          float x, y, z, s, t;
          s = 1.0 - vertex[6];
          t = vertex[7];
          x = vertex[0], y = vertex[1], z = vertex[2];
          glm::vec4 pos(x, y, z, 1.0f);
          glm::vec2 coord(s, t);
          auto glPos = mvp * pos;
          auto glPosPredict = mvpPredict * pos;
          if (visible(glPos)) {
            visibleVertices.push_back(coord);
          } else if (visible(glPosPredict)) {
            predictedVisibleVertices.push_back(coord);
          }
        }
      }
    }
  }

  mtx.unlock();
}

int rendorThreadFunc() {
  // glfw: initialize and configure
  // ------------------------------
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(
      GLFW_OPENGL_FORWARD_COMPAT,
      GL_TRUE);  // uncomment this statement to fix compilation on OS X
#endif

  // glfw window creation
  // --------------------
  GLFWwindow* window =
      glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetKeyCallback(window, key_callback);

  // tell GLFW to capture our mouse
  //   glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // configure global opengl state
  // -----------------------------
  glEnable(GL_DEPTH_TEST);

  // build and compile our shader program
  // ------------------------------------
  Shader ourShader("vertex_shader.glsl", "fragment_shader.glsl");

  // set up vertex data (and buffer(s)) and configure vertex attributes
  // ------------------------------------------------------------------

  // rectangle
  const float verticesRect[] = {
      // positions          // colors           // texture coords
      1.0f,  0.0f, 1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f,  // bottom left
      1.0f,  0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,  // bottom right
      -1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,  // top right
      -1.0f, 0.0f, 1.0f,  1.0f, 1.0f, 0.0f, 1.0f, 0.0f   // top left
  };
  unsigned int indicesRect[] = {
      0, 1, 3,  // first triangle
      1, 2, 3   // second triangle
  };
  const unsigned int lineIndicesRect[] = {0, 1, 1, 3, 3, 0, 1, 2, 2, 3, 3, 1};
  unsigned sizeRect = sizeof(verticesRect);
  unsigned indexSizeRect = sizeof(indicesRect);
  unsigned lineIndexSizeRect = sizeof(lineIndicesRect);
  unsigned strideRect = 8 * sizeof(float);

  unsigned int VBORect, VAORect, EBORect, EBO_linesRect;
  genView(VBORect, VAORect, EBORect, EBO_linesRect, verticesRect, indicesRect,
          lineIndicesRect, sizeRect, indexSizeRect, lineIndexSizeRect,
          strideRect);

  // sphere
  const float* verticesSphere = sphere.getInterleavedVertices();
  const unsigned int* indicesSphere = sphere.getIndices();
  const unsigned int* lineIndicesSphere = sphere.getLineIndices();
  unsigned sizeSphere = sphere.getInterleavedVertexSize();
  unsigned indexSizeSphere = sphere.getIndexSize();
  unsigned lineIndexSizeSphere = sphere.getLineIndexSize();
  unsigned strideSphere = sphere.getInterleavedStride();

  unsigned int VBOSphere, VAOSphere, EBOSphere, EBO_linesSphere;
  genView(VBOSphere, VAOSphere, EBOSphere, EBO_linesSphere, verticesSphere,
          indicesSphere, lineIndicesSphere, sizeSphere, indexSizeSphere,
          lineIndexSizeSphere, strideSphere);

  // load and create a texture
  // -------------------------
  // texture 1
  // ---------
  glGenTextures(1, &texture1);
  glBindTexture(GL_TEXTURE_2D, texture1);

  // set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  // set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
  // // load image, create texture and generate mipmaps
  // int width, height, nrChannels;
  // stbi_set_flip_vertically_on_load(
  //     false);  // tell stb_image.h to flip loaded texture's on the y-axis.
  // unsigned char* data = stbi_load("resources/textures/earth2048.bmp", &width,
  //                                 &height, &nrChannels, 0);
  // if (data) {
  //   glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
  //                GL_UNSIGNED_BYTE, data);
  //   // glGenerateMipmap(GL_TEXTURE_2D);
  // } else {
  //   std::cout << "Failed to load texture" << std::endl;
  // }
  // stbi_image_free(data);

  // tell opengl for each sampler to which texture unit it belongs to (only has
  // to be done once)
  // -------------------------------------------------------------------------------------------
  ourShader.use();
  ourShader.setInt("texture1", 0);

  viewMode = 1;  // sphere by default
  unsigned int VBO, VAO, EBO, EBO_lines;
  unsigned indexSize;
  unsigned lineIndexSize;

  // render loop
  // -----------
  while (!glfwWindowShouldClose(window)) {
    av_usleep(10 * 1e3);

    //   glfwSetWindowShouldClose(window, 1);
    // per-frame time logic
    // --------------------
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    // input
    // -----
    processInput(window);

    // render
    // ------
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // bind textures on corresponding texture units
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture1);
    if (!picQue.empty()) {  // update texture
      auto frame = picQue.pop();
      auto data = frame->data[0];
      texW = frame->width;
      texH = frame->height;
      focusTexS = focusS * texW;
      focusTexT = focusT * texH;

      calculateFoV();

      displayPredict = displayFoV = viewMode != 2 || frozeFoV;
      // draw vertices in FoV
      if (displayFoV) {
        for (auto& pos : visibleVertices) {
          int s = pos.s * texW;
          int t = pos.t * texH;
          int r = texW / numSectors / 4;
          int top, bottom, left, right;
          top = std::max(1, t - r), bottom = std::min(texH - 1, t + r),
          left = (s - r + texW) % texW, right = (s + r) % texW;
          for (int s = left; s != right;
               s = (s + 1) % texW) {  // works even if left>right
            for (int t = top; t != bottom; t = (t + 1) % texH) {
              // int idx = (y * texW + x) * 3;
              int idx = t * frame->linesize[0] + s * 3;
              glm::ivec3 color = glm::ivec3(0, 255, 0);
              for (int i = 0; i < 3; i++) {
                data[idx + i] += std::min(255 - data[idx + i], color[i]);
              }
            }
          }
        }
      }

      if (displayPredict) {
        for (auto& pos : predictedVisibleVertices) {
          int s = pos.s * texW;
          int t = pos.t * texH;
          int r = texW / numSectors / 4;
          int top, bottom, left, right;
          top = std::max(1, t - r), bottom = std::min(texH - 1, t + r),
          left = (s - r + texW) % texW, right = (s + r) % texW;
          for (int s = left; s != right;
               s = (s + 1) % texW) {  // works even if left>right
            for (int t = top; t != bottom; t = (t + 1) % texH) {
              // int idx = (y * texW + x) * 3;
              int idx = t * frame->linesize[0] + s * 3;
              glm::ivec3 color = glm::ivec3(255, 255, 255);
              for (int i = 0; i < 3; i++) {
                data[idx + i] += std::min(255 - data[idx + i], color[i]);
              }
            }
          }
        }
      }

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texW, texH, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, frame->data[0]);
      delete[] frame->data[0];
      av_frame_free(&frame);
    } else {
      // continue;
    }

    // pass projection matrix to shader (note that in this case it could change
    // every frame)
    glm::mat4 projection(1.0f), view(1.0f), model(1.0f);
    // 2、用glm::radians将角度转化为弧度，
    // glm::vec3(1.0f, 0.0f, 0.0f)表示沿X轴旋转
    model =
        glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    if (viewMode == 2) {  // sphere
      VAO = VAOSphere;
      VBO = VBOSphere;
      EBO = EBOSphere;
      EBO_lines = EBO_linesSphere;
      indexSize = indexSizeSphere;
      lineIndexSize = lineIndexSizeSphere;
      projection =
          glm::perspective(glm::radians(camera.Zoom + extraFoV),
                           (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
      // camera/view transformation
      view = camera.GetViewMatrix();
    } else {  // rectangle
      VAO = VAORect;
      VBO = VBORect;
      EBO = EBORect;
      EBO_lines = EBO_linesRect;
      indexSize = indexSizeRect;
      lineIndexSize = lineIndexSizeRect;
    }

    // activate shader
    ourShader.use();
    ourShader.setMat4("projection", projection);
    ourShader.setMat4("view", view);
    ourShader.setMat4("model", model);

    // render
    glBindVertexArray(VAO);
    // set line colour
    // glColor4fv(lineColor);
    // glMaterialfv(GL_FRONT, GL_DIFFUSE,   lineColor);
    // glDrawArrays(GL_TRIANGLES, 0, sphere.getInterleavedVertexCount());
    //  ourShader.setVec4("lineColor", 1.0f, 0.0f, 0.0f, 1.0f);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_lines);
    glDrawElements(GL_LINES, lineIndexSize, GL_UNSIGNED_INT, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, indexSize, GL_UNSIGNED_INT, 0);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved
    // etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this
// frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    camera.ProcessKeyboard(FORWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    camera.ProcessKeyboard(BACKWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    camera.ProcessKeyboard(LEFT, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    camera.ProcessKeyboard(RIGHT, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
    camera.ProcessMouseScroll(deltaTime * zoomSpeed);
  if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
    camera.ProcessMouseScroll(-deltaTime * zoomSpeed);
  }
  float xoffset, yoffset;
  float speed = 16;
  xoffset = yoffset = 0;
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) yoffset += speed;
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) yoffset -= speed;
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) xoffset -= speed;
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) xoffset += speed;

  camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the window size changed (by OS or user resize) this callback
// function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  // make sure the viewport matches the new window dimensions; note that width
  // and height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

static int leftMouseDown = 0;

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
  if (firstMouse) {
    lastX = xpos;
    lastY = ypos;
    firstMouse = false;
  }

  float xoffset = xpos - lastX;
  float yoffset =
      lastY - ypos;  // reversed since y-coordinates go from bottom to top

  lastX = xpos;
  lastY = ypos;

  if (leftMouseDown) {
    camera.ProcessMouseMovement(xoffset, yoffset);
  }
}

void mouse_button_callback(GLFWwindow* window, int button, int action,
                           int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      leftMouseDown = 1;
    } else {
      leftMouseDown = 0;
    }
  }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
  camera.ProcessMouseScroll(yoffset);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action,
                  int mods) {
  // 接受键盘 V 键，更改视图
  if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
    viewMode = (viewMode == 1) ? 2 : 1;
  if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) frozeFoV = !frozeFoV;
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    camera.Position = glm::vec3(0.0f);
}

void updateTexture(u_int8_t* data, int width, int height) {}