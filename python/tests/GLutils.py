# a basic utility for rendering triangle meshes and lines using PyOpenGL

import sys

# rendering requires PyOpenGL & glut
# see https://pypi.org/project/PyOpenGL/ for more details & installation instructions
import OpenGL.GL as GL
import OpenGL.GLU as GLU
import OpenGL.GLUT as GLUT



# shaders source
_vertex_shader_source = '''
uniform mat4 model;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    //gl_FrontColor = gl_Color;

    gl_PointSize = 2.0;
    
    // this should come from uniform?
    vec3 color = vec3(0.4f, 0.4f, 0.4f);
    vec3 lightPos = vec3(-2.0f, 2.0f, 6.0f);
    vec3 viewPos = vec3(0.0, 0.0, 0.0);

    vec4 pos = model * gl_Vertex;
    //vec3 normal = gl_NormalMatrix * gl_Normal;
    vec3 normal = mat3(transpose(inverse(model))) * gl_Normal;  

    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * color;

    // diffuse 
    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPos - pos.xyz);
    float diff = max(dot(norm, lightDir), 0.0);
    //float diff = 0.0f;
    vec3 diffuse = diff * color;

    // specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - pos.xyz);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * color;  

    vec3 r = ambient + diffuse + specular;
    gl_FrontColor = vec4(r, 1.0f);
    //gl_FrontColor = vec4(color, 1.0f);
}
'''
_fragment_shader_source = '''
void main()
{
    gl_FragColor = gl_Color;
}
'''


# global list of renderable data, see helpers below on how to use it
g_renderables = []

# helper methods to add renderable data
def addPointSet(points):
    ''' Add a set of 3D points to be rendered, input should be a flat list of coords with size a multiple of 3'''
    global g_renderables
    g_renderables.append({"type":"points", "positions":points})

def addLineSet(lines):
    ''' Add a set of 3D lines to be rendered, each line is defined by two consecutive points in the input 
        flat list of coords
    '''
    global g_renderables
    g_renderables.append({"type":"lines", "positions":lines})

def addTriMesh(vertices, normals, indices):
    global g_renderables
    g_renderables.append({"type":"trimesh", "vertices":vertices, "normals":normals, "indices":indices})

# local variables controlling the camera position and render state
g_yaw = 0
g_pitch = 0
g_translation_offset = 0
g_translation_step = 0.1
g_active_renderable = -1    # -1 means render all


def printOpenGLError():
    err = GL.glGetError()
    if (err != GL.GL_NO_ERROR):
        print('GLERROR: ', GLU.gluErrorString(err))
        sys.exit(-1)

def createShaderGL():
    global program

    # create program
    program = GL.glCreateProgram()
    #print('create program')
    printOpenGLError()

    # vertex shader
    #print('compile vertex shader...')
    vs = GL.glCreateShader(GL.GL_VERTEX_SHADER)
    GL.glShaderSource(vs, [_vertex_shader_source])
    GL.glCompileShader(vs)
    GL.glAttachShader(program, vs)
    printOpenGLError()

    # fragment shader
    #print('compile fragment shader...')
    fs = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
    GL.glShaderSource(fs, [_fragment_shader_source])
    GL.glCompileShader(fs)
    GL.glAttachShader(program, fs)
    printOpenGLError()

    #print('link...')
    GL.glLinkProgram(program)
    printOpenGLError()

    #return program

def initGL():
    GL.glClearColor(0.4, 0.7, 1.0, 0.0)
    GL.glClearDepth(1.0)
    GL.glDepthFunc(GL.GL_LESS)
    GL.glEnable(GL.GL_DEPTH_TEST)

def resizeGL(Width, Height):
    if (Height == 0):
        Height = 1
    GL.glViewport(0, 0, Width, Height)
    GL.glMatrixMode(GL.GL_PROJECTION)
    GL.glLoadIdentity()
    GLU.gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)

def keyPressed(key, x, y):
    global g_yaw, g_pitch, g_translation_offset, g_active_renderable

    # Key       Desc
    # ----------------------------------------------------------
    # W/S       camera forwards/backwards movement
    # A/D       camera left/right rotation around rendered mesh
    # ESC       close window
    # R         reset to render everything
    # T/G       show next/previous render data

    asciicode = int.from_bytes(key, "little")

    # If escape is pressed, kill everything.
    if asciicode == 27:   # 'ESCAPE' 
        sys.exit()
    elif asciicode == 97: # a
        g_yaw += 0.8
    elif asciicode == 100: # d
        g_yaw -= 0.8
    elif asciicode == 115: # s
        g_translation_offset += g_translation_step
    elif asciicode == 119: # w
        g_translation_offset -= g_translation_step
    elif asciicode == 114: # r
        g_active_renderable = -1# * g_active_renderable
    elif asciicode == 116: # t
        g_active_renderable = (g_active_renderable + 1) % len(g_renderables)
    elif asciicode == 103: # g
        g_active_renderable = abs(g_active_renderable - 1)
    

def draw_points(points):
    #global points

    if not points:
        return

    GL.glPointSize(4.0)
    #GL.glLineWidth(5.0)
    GL.glColor3f(1.0, 0.0, 0.0)
    #GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)

    GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
    GL.glVertexPointer(3, GL.GL_FLOAT, 0, points)
    pts_count = int(len(points)/3)
    GL.glDrawArrays(GL.GL_POINTS, 0, pts_count)
    GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
    
    #GL.glDisable(GL.GL_PROGRAM_POINT_SIZE)
    #printOpenGLError()

def draw_lines(lines):
    if lines == None or len(lines) == 0:
        return

    GL.glPointSize(1.5)
    #GL.glLineWidth(5.0)
    GL.glColor3d(1.0, 0.0, 1.0)

    GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
    GL.glVertexPointer(3, GL.GL_FLOAT, 0, lines)
    # draw both points and lines for line pairs
    pts_count = int(len(lines)/3)
    GL.glDrawArrays(GL.GL_POINTS, 0, pts_count)
    #lines_count = int(pts_count/2)
    #print(lines_count)
    GL.glDrawArrays(GL.GL_LINES, 0, pts_count)
    GL.glDisableClientState(GL.GL_VERTEX_ARRAY)

    #printOpenGLError()

def draw_tri_mesh(vertices, normals, indices):
    global program

    if len(vertices) > 0:#vertices:# and normals and indices:
        if GL.glUseProgram(program):
            printOpenGLError()
        
        mid = GL.glGetUniformLocation(program, "model")
        mtx = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -4.0, 1.0]
        # ignore rotation for simplicity
        GL.glUniformMatrix4fv(mid, 1, GL.GL_FALSE, mtx)

        #draw_vbo()
        GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
        #GL.glEnableClientState(GL.GL_COLOR_ARRAY)
        GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
        GL.glVertexPointer(3, GL.GL_FLOAT, 0, vertices)
        #GL.glColorPointer(3, GL.GL_FLOAT, 0, colors)
        GL.glNormalPointer(GL.GL_FLOAT, 0, normals)
        GL.glDrawElements(GL.GL_TRIANGLES, len(indices), GL.GL_UNSIGNED_INT, indices)
        #GL.glDisableClientState(GL.GL_COLOR_ARRAY)
        GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
        GL.glDisableClientState(GL.GL_VERTEX_ARRAY)

        GL.glUseProgram(0)

def onDrawGL():
    global g_yaw, g_pitch, g_translation_offset
    
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

    # test
    GL.glMatrixMode(GL.GL_MODELVIEW)
    GL.glLoadIdentity()
    #g_yaw+=0.05
    #g_pitch+=0.05
    GL.glTranslatef(0.0, 0.0, -g_translation_offset)
    GL.glRotatef(g_yaw, 0, 1, 0)
    #GL.glRotatef(g_pitch, 1, 0, 0)

    #GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_POINT) 
    #GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE) 

    for i, renderable in enumerate(g_renderables):
        if g_active_renderable >= 0 and g_active_renderable != i:
            continue
        if renderable["type"] == "trimesh":
            draw_tri_mesh(renderable["vertices"], renderable["normals"], renderable["indices"])
        elif renderable["type"] == "points":
            draw_points(renderable["positions"])
        elif renderable["type"] == "lines":
            draw_lines(renderable["positions"])

    GL.glFlush()
    GLUT.glutSwapBuffers()

def createGLUTWindow(windowTitle):
    GLUT.glutInit(sys.argv)
    GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA | GLUT.GLUT_DOUBLE | GLUT.GLUT_DEPTH)

    GLUT.glutInitWindowSize(1024, 768)
    GLUT.glutInitWindowPosition(120, 120)
    GLUT.glutCreateWindow(windowTitle)

    GLUT.glutDisplayFunc(onDrawGL)
    GLUT.glutIdleFunc(onDrawGL)
    GLUT.glutReshapeFunc(resizeGL)
    GLUT.glutKeyboardFunc(keyPressed)

    initGL()
    createShaderGL()

def startLoop():
    GLUT.glutMainLoop()