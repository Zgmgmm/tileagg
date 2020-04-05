
#include <iostream>
#include <thread>

#include "glog/logging.h"
#include "glad/glad.h"  // glad必须在GLFW/GL之前include
#include "GL/gl.h"
#include "GLFW/glfw3.h"
#include "Sphere.h"
#include "camera.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "shader_m.h"
#include "BlockingQueue.h"
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

const float PI = acos(-1);

void startRendor();
int rendorThreadFunc();
void getFoV(int w, int h, int& x, int& y, int& left, int& right, int& top, int& bottom);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action,
                           int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// queue
extern BlockingQueue<AVFrame*> picQue;
// thread
thread* rendorThread;

// texture
unsigned int texture1;

// settings
const unsigned int SCR_WIDTH = 640;
const unsigned int SCR_HEIGHT = 360;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 0.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// sphere
Sphere sphere(1.0f, 144,
              72);  // radius, sectors, stacks, non-smooth (flat) shading

// timing
float deltaTime = 0.0f;  // time between current frame and last frame
float lastFrame = 0.0f;

void startRendor() {
  cout << "rendor thread started." << endl;
  rendorThread = new thread(rendorThreadFunc);
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

#ifndef GL_RENDOR_SPHERE
  // rectangle
  const float vertices[] = {
      // positions          // colors           // texture coords
      1.0f,  0.0f, 1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f,  // bottom left
      1.0f,  0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,  // bottom right
      -1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,  // top right
      -1.0f, 0.0f, 1.0f,  1.0f, 1.0f, 0.0f, 1.0f, 0.0f   // top left
  };
  unsigned int indices[] = {
      0, 1, 3,  // first triangle
      1, 2, 3   // second triangle
  };
  const unsigned int lineIndices[] = {0, 1, 1, 3, 3, 0, 1, 2, 2, 3, 3, 1};
  unsigned size = sizeof(vertices);
  unsigned indexSize = sizeof(indices);
  unsigned lineIndexSize = sizeof(lineIndices);
  unsigned stride = 8 * sizeof(float);
  camera.Position = glm::vec3(0.0f, 0.0f, 1.0f);
#else
  // sphere
  const float* vertices = sphere.getInterleavedVertices();
  const unsigned int* indices = sphere.getIndices();
  const unsigned int* lineIndices = sphere.getLineIndices();
  unsigned size = sphere.getInterleavedVertexSize();
  unsigned indexSize = sphere.getIndexSize();
  unsigned lineIndexSize = sphere.getLineIndexSize();
  unsigned stride = sphere.getInterleavedStride();
#endif

  unsigned int VBO, VAO, EBO, EBO_lines;
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

  // render loop
  // -----------
  while (!glfwWindowShouldClose(window)) {
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
      auto w = frame->width;
      auto h = frame->height;
      // modify frame
      int x, y, left, right, top, bottom;
      getFoV(w, h, x, y, left, right, top, bottom);
      for (int x = left; x != right;
           x = (x + 1) % w) {  // works even if left>right
        for (int y = top; y != bottom; y = (y + 1) % h) {
          int idx = y * frame->linesize[0] + x * 3;
          for (int i = idx; i < idx + 3; i++) {
            data[i] += std::min(255 - data[i], 32);
          }
        }
      }

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE,
                   data);
      delete[] frame->data[0];
      av_frame_free(&frame);
    }

    // activate shader
    ourShader.use();


    // pass projection matrix to shader (note that in this case it could change
    // every frame)
    glm::mat4 projection(1.0f), view(1.0f);
#ifdef GL_RENDOR_SPHERE
    projection =
        glm::perspective(glm::radians(camera.Zoom),
                         (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    // camera/view transformation
    view = camera.GetViewMatrix();
#endif
    ourShader.setMat4("projection", projection);
    ourShader.setMat4("view", view);

    // render boxes
    glBindVertexArray(VAO);
    // for (unsigned int i = 0; i < 10; i++) {
    //   // calculate the model matrix for each object and pass it to shader
    //   before
    //   // drawing
    glm::mat4 model = glm::mat4(
        1.0f);  // make sure to initialize matrix to identity matrix first
                //   model = glm::translate(model, cubePositions[i]);
                //   float angle = 20.0f * i;
                //   model =
    // 2、用glm::radians将角度转化为弧度，glm::vec3(1.0f, 0.0f,
    // 0.0f)表示沿X轴旋转
    model =
        glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    ourShader.setMat4("model", model);

    // set line colour
    // glColor4fv(lineColor);
    // glMaterialfv(GL_FRONT, GL_DIFFUSE,   lineColor);
    // glDrawArrays(GL_TRIANGLES, 0, sphere.getInterleavedVertexCount());
    //  ourShader.setVec4("lineColor", 1.0f, 0.0f, 0.0f, 1.0f);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_lines);
    glDrawElements(GL_LINES, lineIndexSize, GL_UNSIGNED_INT, 0);

    ourShader.setVec4("lineColor", 0.0f, 0.0f, 0.0f, 0.0f);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, indexSize, GL_UNSIGNED_INT, 0);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved
    // etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();

    av_usleep(10 * 1e3);
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

    const auto& front = camera.Front; 

    // input: WxH, pitch, yaw, fov
    // output: x, y, left, right, top, bottom(center and bound)

    float h, w;
    int x, y, left, right, top, bottom;

    w = 1280, h = 720;

    getFoV(w, h, x, y, left, right, top, bottom);

    // camera.getFoV(1.0f, x, y, left, right, top, bottom);
    // x = x * w + 0.5f, y = y * h + 0.5f, left = left * w + 0.5f,
    // right = right * w + 0.5f, top = top * h + 0.5f, bottom = bottom * h + 0.5f;

    char buf[512];
    sprintf(buf,
            // "x=%.f y=%.f l=%.f r=%.f t=%.f b=%.f "
            "x=%d y=%d l=%d r=%d t=%d b=%d "
            "Yaw=%.3f "
            "Pitch=%.3f front(%.3f, "
            "%.3f, %.3f)",
            x, y, left, right, top, bottom, camera.Yaw, camera.Pitch, front.x,
            front.y, front.z);
    LOG(INFO) << buf << endl;
  }
}

void getFoV(int w, int h, int& x, int& y, int& left, int& right, int& top, int& bottom) {
  float tx, ty, l, r, t, b;
  camera.getFoV(1.0f, tx, ty, l, r, t, b);
  x = tx * w + 0.5f, y = ty * h + 0.5f, left = l * w + 0.5f,
  right = r * w + 0.5f, top = t * h + 0.5f, bottom = b * h + 0.5f; 
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

void updateTexture(u_int8_t* data, int width, int height) {}