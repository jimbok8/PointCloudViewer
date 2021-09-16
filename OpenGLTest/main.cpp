#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

void InitWindow(int, char* []);
void Initialize(int, char* []);
void CreateShaders(void);
void CreateTexture(void);

GLuint programId, computeShaderId;
GLuint inputTexId, outputTexId;

const int kArraySize = 1000;

const GLchar* Program = " \
    #version 430\n\
    layout (local_size_x = 16, local_size_y = 16) in;\n\
    layout (r32f, binding = 0) uniform image1D in_array; \n\
    layout (r32f, binding = 1) uniform image1D out_array; \n\
    \
    void main() \n\
    { \
        int pos = int(gl_GlobalInvocationID.x);\n\
        vec4 value = imageLoad(in_array, pos);\n\
        value.x += 1.0f;\n\
        imageStore(out_array, pos, value);\n\
    } \
";

void CheckGLErrors()
{
}

// 创建opengl的窗口
void InitWindow(int argc, char* argv[])
{
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
}

void Initialize(int argc, char* argv[])
{
    GLenum GlewInitResult;

    InitWindow(argc, argv);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }

    CreateShaders();
    CreateTexture();
}

void CreateShaders(void)
{
    GLchar messages[256];
    GLenum ErrorCheckValue = glGetError();

    /* Compile the shader. */
    computeShaderId = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(computeShaderId, 1, &Program, NULL);
    glCompileShader(computeShaderId);

    /* Print the compilation log. */
    glGetShaderInfoLog(computeShaderId, sizeof(messages), NULL, messages);
    printf("Compile Log: %s\n", messages);

    /* Set up program objects. */
    programId = glCreateProgram();

    /* Create a complete program object. */
    glAttachShader(programId, computeShaderId);
    glLinkProgram(programId);

    /* And print the link log. */
    glGetProgramInfoLog(programId, sizeof(messages), NULL, messages);
    printf("Link Log: %s\n", messages);

    CheckGLErrors();
}

void CreateTexture(void)
{
    // Create the input texture
    glGenTextures(1, &inputTexId);

    // And bind it to texture unit 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_1D, inputTexId);
    // Set texture size and format
    glTexStorage1D(GL_TEXTURE_1D, 1, GL_R32F, kArraySize);

    // Create the output texture
    glGenTextures(1, &outputTexId);

    // And bind it to texture unit 1
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_1D, outputTexId);
    // Set texture size and format
    glTexStorage1D(GL_TEXTURE_1D, 1, GL_R32F, kArraySize);

    glBindImageTexture(0, inputTexId, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
    glBindImageTexture(1, outputTexId, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);

    CheckGLErrors();
}

void DoCompute()
{
    float* inputData = new float[kArraySize];
    float* outputData = new float[kArraySize];

    int i;
    for (i = 0; i < kArraySize; i++)
        inputData[i] = i;

    glBindTexture(GL_TEXTURE_1D, inputTexId);
    glTexSubImage1D(GL_TEXTURE_1D, 0, 0, kArraySize, GL_RED, GL_FLOAT, inputData);


    // launch compute shaders!
    glUseProgram(programId);
    glDispatchCompute((GLuint)kArraySize / 16, 1, 1);

    // make sure writing to image has finished before read
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    glBindTexture(GL_TEXTURE_1D, outputTexId);
    glGetTexImage(GL_TEXTURE_1D, 0, GL_RED, GL_FLOAT, outputData);
    glBindTexture(GL_TEXTURE_2D, 0);

    CheckGLErrors();

    for (i = 0; i < kArraySize; i++)
        printf("%f ", outputData[i]);
    
    delete[]outputData;
    delete[]inputData;
}

int main(int argc, char* argv[])
{
    Initialize(argc, argv);

    DoCompute();
    //glutMainLoop();

    return(0);
}