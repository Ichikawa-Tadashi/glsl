#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if defined(WIN32)
//#  pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#  include "glut.h"
#  include "glext.h"
#elif defined(__APPLE__) || defined(MACOSX)
#  include <GLUT/glut.h>
#else
#  define GL_GLEXT_PROTOTYPES
#  include <GL/glut.h>
#endif




// https://stackoverflow.com/questions/76985642/is-there-a-way-to-set-absolute-cursor-position-for-uinput-virtual-device-waylan

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/uinput.h>
#include <sys/ioctl.h>

static void fatal(const char *msg)
{
    fprintf(stderr, "fatal: ");

    if (errno)
        perror(msg);
    else
        fprintf(stderr, "%s\n", msg);

    exit(EXIT_FAILURE);
}

static void setup_abs(int fd, __u16 type, int min, int max, int res)
{
    struct uinput_abs_setup abs = {
        .code = type,
        .absinfo = {
            .minimum = min,
            .maximum = max,
            .resolution = res
        }
    };

    if (-1 == ioctl(fd, UI_ABS_SETUP, &abs))
        fatal("ioctl UI_ABS_SETUP");
}

static void mouse_init(int fd, int width, int height, int dpi)
{
    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_SYN))
        fatal("ioctl UI_SET_EVBIT EV_SYN");

    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_KEY))
        fatal("ioctl UI_SET_EVBIT EV_KEY");
    if (-1 == ioctl(fd, UI_SET_KEYBIT, BTN_LEFT))
        fatal("ioctl UI_SET_KEYBIT BTN_LEFT");

    if (-1 == ioctl(fd, UI_SET_EVBIT, EV_ABS))
        fatal("ioctl UI_SET_EVBIT EV_ABS");
    /* the ioctl UI_ABS_SETUP enables these automatically, when appropriate:
        ioctl(fd, UI_SET_ABSBIT, ABS_X);
        ioctl(fd, UI_SET_ABSBIT, ABS_Y);
    */

    struct uinput_setup device = {
        .id = {
            .bustype = BUS_USB
        },
        .name = "Emulated Absolute Positioning Device"
    };

    if (-1 == ioctl(fd, UI_DEV_SETUP, &device))
        fatal("ioctl UI_DEV_SETUP");

    setup_abs(fd, ABS_X, 0, width, dpi);
    setup_abs(fd, ABS_Y, 0, height, dpi);

    if (-1 == ioctl(fd, UI_DEV_CREATE))
        fatal("ioctl UI_DEV_CREATE");

    /* give time for device creation */
    sleep(1);
}

static void mouse_emit(int fd, __u16 type, __u16 code, int value)
{
    struct input_event ie = {
        .type = type,
        .code = code,
        .value = value
    };

    write(fd, &ie, sizeof ie);
}

int mouse_fd;
// int main(int argc, char **argv)
// {
//     /* These values are very device specific */
//     int w = argc > 1 ? atoi(argv[1]) : 1920;
//     int h = argc > 2 ? atoi(argv[2]) : 1080;
//     int d = argc > 3 ? atoi(argv[3]) : 96;

//     if (w < 1 || h < 1 || d < 1)
//         fatal("Bad initial value(s).");

//     mouse_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

//     if (-1 == mouse_fd)
//         fatal("open");

//     printf("Initializing device screen map as %dx%d @ %ddpi\n", w, h, d);

    // init(mouse_fd, w, h, d);

    // while (1) {
        // printf("Enter x & y: ");
        // fflush(stdout);

        // char input[128];
        // if (!fgets(input, sizeof input, stdin) || 0 == strncmp(".exit", input, 5))
        //     break;

        // int x, y;
        // if (2 != sscanf(input, "%d%d", &x, &y) || x < 0 || y < 0) {
        //     fprintf(stderr, "Invalid input.\n");
        //     continue;
        // }

        // printf("Moving cursor to %d,%d\n", x, y);

        /* input is zero-based, but event positions are one-based */
        // emit(mouse_fd, EV_ABS, ABS_X, 1 + x);
        // emit(mouse_fd, EV_ABS, ABS_Y, 1 + y);
        // emit(mouse_fd, EV_SYN, SYN_REPORT, 0);
    // }

    // puts("Cleaning up...");

    // /* give time for events to finish */
    // sleep(1);

    // if (-1 == ioctl(mouse_fd, UI_DEV_DESTROY))
    //     fatal("ioctl UI_DEV_DESTROY");

    // close(mouse_fd);
    // puts("Goodbye.");
// }





char *fragshadername;

/*
** ����
*/
static const GLfloat lightpos[] = { 0.0f, 0.0f, 5.0f, 1.0f }; /* �ʒu�@�@�@ */
static const GLfloat lightcol[] = { 1.0f, 1.0f, 1.0f, 1.0f }; /* ���ڌ����x */
static const GLfloat lightamb[] = { 0.1f, 0.1f, 0.1f, 1.0f }; /* �������x */

/*
** �V�F�[�_
*/
#include "glsl.h"
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;

int windowX,windowY;
int width=640,height=480;
float sens=0.18; // rad:dX/1000.0*sens
int mouseX,mouseY;
float lookX=800.0,lookY=500.0; //max 1000.0
float cursorX,cursorY;
int TIMER_INTERVAL=10; //millisecond
float time=0.0; //second

static GLfloat camerapos[] = { -1.5f, 1.7f, -1.4f, 1.0f };
static GLfloat cameraforward[] = { 0.0f, 0.0f, 1.0f, 1.0f };
static GLfloat cameraleft[] = { 1.0f, 0.0f, 0.0f, 1.0f };
static GLfloat cameraang[] = { 0.0f, 0.0f };

static GLfloat cameraacc[] = { 0.0, 0.0, 0.0 };
static GLfloat gravity = 0.5;

bool keystate[256];
char keymodifiers;
bool isPaused;

/*
** ������
*/
static void init(void)
{
  /* �V�F�[�_�v���O�����̃R���p�C���^�����N���ʂ𓾂�ϐ� */
  GLint compiled, linked;

  /* �����ݒ� */
  glClearColor(0.3f, 0.3f, 1.0f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  /* �����̏����ݒ� */
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightcol);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightcol);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

  /* GLSL �̏����� */
  if (glslInit()) exit(1);

  /* �V�F�[�_�I�u�W�F�N�g�̍쐬 */
  vertShader = glCreateShader(GL_VERTEX_SHADER);
  fragShader = glCreateShader(GL_FRAGMENT_SHADER);

  /* �V�F�[�_�̃\�[�X�v���O�����̓ǂݍ��� */
  if (readShaderSource(vertShader, "simple.vert")) exit(1);
  if (readShaderSource(fragShader, fragshadername)) exit(1);

  /* �o�[�e�b�N�X�V�F�[�_�̃\�[�X�v���O�����̃R���p�C�� */
  glCompileShader(vertShader);
  glGetShaderiv(vertShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(vertShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in vertex shader.\n");
    exit(1);
  }

  /* �t���O�����g�V�F�[�_�̃\�[�X�v���O�����̃R���p�C�� */
  glCompileShader(fragShader);
  glGetShaderiv(fragShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(fragShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in fragment shader.\n");
    exit(1);
  }

  /* �v���O�����I�u�W�F�N�g�̍쐬 */
  gl2Program = glCreateProgram();

  /* �V�F�[�_�I�u�W�F�N�g�̃V�F�[�_�v���O�����ւ̓o�^ */
  glAttachShader(gl2Program, vertShader);
  glAttachShader(gl2Program, fragShader);

  /* �V�F�[�_�I�u�W�F�N�g�̍폜 */
  glDeleteShader(vertShader);
  glDeleteShader(fragShader);

  /* �V�F�[�_�v���O�����̃����N */
  glLinkProgram(gl2Program);
  glGetProgramiv(gl2Program, GL_LINK_STATUS, &linked);
  printProgramInfoLog(gl2Program);
  if (linked == GL_FALSE) {
    fprintf(stderr, "Link error.\n");
    exit(1);
  }

  /* �V�F�[�_�v���O�����̓K�p */
  glUseProgram(gl2Program);
}

/*
** �V�[���̕`��
*/
static void scene(void)
{
  static const GLfloat diffuse[] = { 0.6f, 0.1f, 0.1f, 1.0f };
  static const GLfloat specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };

  /* �ގ��̐ݒ� */
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0f);

#if 1
  /* �P���̂S�p�`��`�� */
  glNormal3d(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  // glVertex3d(-1.0, -1.0,  0.0);
  // glVertex3d( 1.0, -1.0,  0.0);
  // glVertex3d( 1.0,  1.0,  0.0);
  // glVertex3d(-1.0,  1.0,  0.0);

  glVertex3d(-2.4, -2.0,  0.0);
  glVertex3d( 2.4, -2.0,  0.0);
  glVertex3d( 2.4,  2.0,  0.0);
  glVertex3d(-2.4,  2.0,  0.0);

  glEnd();
#else
  glutSolidTeapot(1.0);
#endif
}


static void display(void)
{
  /* ���f���r���[�ϊ��s��̏����� */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /* ��ʃN���A */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* �����̈ʒu��ݒ� */
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

  /* ���_�̈ړ��i���̂̕������Ɉړ��j*/
  glTranslated(0.0, 0.0, -3.0);

  /* �V�[���̕`�� */
  scene();

  /* �_�u���o�b�t�@�����O */
  glutSwapBuffers();
}

//フラグメントシェーダのunform変数に値を渡す
void sendTimer(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "time");
  glUniform1f(id_f,time);
  glutPostRedisplay();
}
void sendResolution(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "resolution");
  glUniform2f(id_f,width*1.0,height*1.0);
  glutPostRedisplay();
}
void sendMousePos(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "mouse_pos");
  glUniform2f(id_f,mouseX*1.0,mouseY*1.0);
  glutPostRedisplay();
}
void sendCursorPos(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "cursor_pos");
  glUniform2f(id_f,cursorX*1.0,cursorY*1.0);
  glutPostRedisplay();
}

void sendCameraAng(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "camera_ang");
  glUniform2f(id_f,cameraang[0],cameraang[1]);
  // glUniform2fv(id_f,2,cameraang);
  glutPostRedisplay();
}
void sendCameraPos(){
  GLint id_f;
  id_f = glGetUniformLocation(gl2Program, "CameraPos");
  glUniform3f(id_f,camerapos[0],camerapos[1],camerapos[2]);
  // glUniform2fv(id_f,2,cameraang);
  glutPostRedisplay();
}

//回転行列、行列とベクトルの積を定義
GLfloat* x_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    GLfloat* ret=(GLfloat*)malloc(sizeof(GLfloat)*9);
    GLfloat cp[9]={1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c};
    for(int i=0; i<9; i++)ret[i]=cp[i];
    return ret;
}

GLfloat* y_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    GLfloat* ret=(GLfloat*)malloc(sizeof(GLfloat)*9);
    GLfloat cp[9]={c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0,  c};
    for(int i=0; i<9; i++)ret[i]=cp[i];
    return ret;
}

GLfloat* z_axis_rot(float angle){
    float c = cos(angle);
    float s = sin(angle);
    GLfloat* ret=(GLfloat*)malloc(sizeof(GLfloat)*9);
    GLfloat cp[9]={c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0};
    for(int i=0; i<9; i++)ret[i]=cp[i];
    return ret;
}

GLfloat* mul3d_mat_vec(GLfloat* mat, GLfloat* vec){
  GLfloat mat4x4[4][4]={0};mat4x4[3][3]=1.0;
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      mat4x4[i][j]=mat[i*3+j];
    }
  }
  GLfloat vec4d[4]={0};vec4d[3]=1.0;
  for(int i=0; i<3; i++){vec4d[i]=vec[i];}

  GLfloat ans4d[4]={0};
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      ans4d[i]+=vec4d[j]*mat4x4[i][j];
    }
  }

  GLfloat* ans3d=(GLfloat*)malloc(3*sizeof(GLfloat));
  for(int i=0; i<3; i++){ans3d[i]=ans4d[i]/ans4d[3];}
  return ans3d;
}

int k=0;
static void timer(int id){
  // cameraang[0]=(830.0/1000.0-0.5)*2.0*M_PI;
  // cameraang[1]=-0.5*M_PI+(680.0/1000.0-0.5)*M_PI;
  //   sendCameraPos();
  //   sendCameraAng();
    
  //   time+=0.1;sendTimer(); //TODO
  if(id==0&&!isPaused){ //TODO
    //timer loop
    time+=0.1;
    sendTimer();

      
  windowX=glutGet(GLUT_WINDOW_X);
  windowY=glutGet(GLUT_WINDOW_Y);
  int toX=windowX+width/2+(k%2),toY=windowY+height/2+(k%4)/2; //毎回場所を少しずらさないと作動しない
  int dX=(windowX+mouseX)-toX,dY=(windowY+mouseY)-toY;
  if(abs(dX)>1){lookX+=dX*sens;}
  if(abs(dY)>1){lookY+=dY*sens;}
  if(lookY<0)lookY=0;
  if(lookY>1000.0)lookY=1000.0;
  

        k++;
  // moveMouse(dX,dY);
        fflush(stdout);
        // printf("%d %d\n",lookX,lookY);
        // printf("%d %d\n",dX,dY);
        mouse_emit(mouse_fd, EV_ABS, ABS_X, 1 + toX);
        mouse_emit(mouse_fd, EV_ABS, ABS_Y, 1 + toY);
        mouse_emit(mouse_fd, EV_SYN, SYN_REPORT, 0);
        // usleep(100000);

    
  // cameraang[0]=(mouseX*1.0/width-0.5)*2.0*M_PI;
  // cameraang[1]=-0.5*M_PI+(mouseY*1.0/height-0.5)*M_PI;
  cameraang[0]=(lookX/1000.0-0.5)*2.0*M_PI;
  cameraang[1]=-0.5*M_PI+(lookY/1000.0-0.5)*M_PI;

    //なぜかzrotの修正必要
    GLfloat* rotatedforward=mul3d_mat_vec(z_axis_rot(-cameraang[0] *-1.0 + M_PI),
      mul3d_mat_vec(//x_axis_rot(-cameraang[1]) //←向いてる方向に移動 ↓上下は変えない
        x_axis_rot(0.5*M_PI),cameraforward));
    
    GLfloat* rotatedleft=mul3d_mat_vec(z_axis_rot(-cameraang[0] *-1.0 + M_PI),
      mul3d_mat_vec(//x_axis_rot(-cameraang[1])
        x_axis_rot(0.5*M_PI),cameraleft));
    
    // float MOVE_SPEED=0.015;
    float MOVE_SPEED=0.04;
    if(keymodifiers & GLUT_ACTIVE_SHIFT){
      MOVE_SPEED=0.02;
    }

    GLfloat cameraacc_xy[2]={0.0,0.0};
    if(keystate['a']){
      cursorX-=1.0*TIMER_INTERVAL/1000.0;
      // for(int i=0; i<3; i++)camerapos[i]+=rotatedleft[i]*MOVE_SPEED;
      for(int i=0; i<2; i++)cameraacc_xy[i]+=rotatedleft[i];
    }
    if(keystate['d']){
      cursorX+=1.0*TIMER_INTERVAL/1000.0;
      // for(int i=0; i<3; i++)camerapos[i]-=rotatedleft[i]*MOVE_SPEED;
      for(int i=0; i<2; i++)cameraacc_xy[i]-=rotatedleft[i];
    }
    if(keystate['w']){
      cursorY-=1.0*TIMER_INTERVAL/1000.0;
      // for(int i=0; i<3; i++)camerapos[i]+=rotatedforward[i]*MOVE_SPEED;
      for(int i=0; i<2; i++)cameraacc_xy[i]+=rotatedforward[i];
    }
    if(keystate['s']){
      cursorY+=1.0*TIMER_INTERVAL/1000.0;
      // for(int i=0; i<3; i++)camerapos[i]-=rotatedforward[i]*MOVE_SPEED;
      for(int i=0; i<2; i++)cameraacc_xy[i]-=rotatedforward[i];
    }

    double acc_abs=sqrt(cameraacc_xy[0]*cameraacc_xy[0]+cameraacc_xy[1]*cameraacc_xy[1]);
    if(acc_abs>0.0){
    for(int i=0; i<2; i++)cameraacc_xy[i]*=MOVE_SPEED/acc_abs;
    for(int i=0; i<2; i++)camerapos[i]+=cameraacc_xy[i];
    }
    
    for(int i=0; i<3; i++)camerapos[i]+=cameraacc[i];

    cameraacc[2]+=gravity*TIMER_INTERVAL/1000.0;
    if(camerapos[2]>=0.0-0.01){
      camerapos[2]=0.0;
      cameraacc[2]=0.0;
      if(keystate[' ']){
        cameraacc[2]=-0.08;
      }
    }

    sendCursorPos();
    sendCameraPos();
    sendCameraAng();
  }
  
  glutTimerFunc(TIMER_INTERVAL,timer,0);
}

static void resize(int w, int h){
  /* �E�B���h�E�S�̂��r���[�|�[�g�ɂ��� */
  glViewport(0, 0, w, h);

  /* �����ϊ��s��̎w�� */
  glMatrixMode(GL_PROJECTION);

  /* �����ϊ��s��̏����� */
  glLoadIdentity();
  gluPerspective(60.0, (double)w / (double)h, 1.0, 100.0);

  width=w;height=h;
  sendResolution();
}

static void idle(void)
{
  /* ��ʂ̕`���ւ� */
  glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y)
{
  mouseX=x;mouseY=y;
  sendMousePos();
  // cameraang[0]=(mouseX*1.0/width-0.5)*2.0*M_PI;
  // cameraang[1]=-0.5*M_PI+(mouseY*1.0/height-0.5)*M_PI;
  // sendCameraAng();

  switch (button) {
  case GLUT_LEFT_BUTTON:
    switch (state) {
    case GLUT_DOWN:
      glutIdleFunc(idle);
      break;
    case GLUT_UP:
      glutIdleFunc(0);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

// void glutWarpPointer(int x, int y)
// {
// XWarpPointer(__glutDisplay, None, __glutCurrentWindow->win,
// 0, 0, 0, 0, x, y);
// XFlush(__glutDisplay);


    // Display *displayMain = XOpenDisplay(NULL);

    // if(displayMain == NULL)
    // {
    //     fprintf(stderr, "Errore nell'apertura del Display !!!\n");
    //     exit(EXIT_FAILURE);
    // }

    // XWarpPointer(displayMain, None, None, 0, 0, 0, 0, x, y);

    // XCloseDisplay(displayMain);
// }


static void motion(int x, int y)
{
  
  // static bool wrap = false;
  // if((!wrap)||true){
  // windowX=glutGet(GLUT_WINDOW_X);
  // windowY=glutGet(GLUT_WINDOW_Y);
  // int toX=windowX+width/2,toY=windowY+height/2;
  // int dX=toX-x,dY=toY-y;
  // // moveMouse(dX,dY);
  //       fflush(stdout);
  //       printf("%d %d\n",toX,toY);
  //       mouse_emit(mouse_fd, EV_ABS, ABS_X, 1 + toX);
  //       mouse_emit(mouse_fd, EV_ABS, ABS_Y, 1 + toY);
  //       mouse_emit(mouse_fd, EV_SYN, SYN_REPORT, 0);
  //       // usleep(100000);
  //   wrap=true;
  // }else{
  //   wrap=false;
  // }

  mouseX=x;mouseY=y;
  // mouseX+=x-(width/2);mouseY+=y-(height/2);
  sendMousePos();
  // cameraang[0]=(mouseX*1.0/width-0.5)*2.0*M_PI;
  // cameraang[1]=-0.5*M_PI+(mouseY*1.0/height-0.5)*M_PI;
  // sendCameraAng();
  // glutWarpPointer(width / 2, height / 2);
  // printf("%d %d\n",width,height);
  // /* �g���b�N�{�[���ړ� */

  // static bool wrap = false;
  // if(!wrap) {
  //   int ww = glutGet(GLUT_WINDOW_WIDTH);
  //   int wh = glutGet(GLUT_WINDOW_HEIGHT);

  //   int dx = x - ww / 2;
  //   int dy = y - wh / 2;

  //   // Do something with dx and dy here

  //   // move mouse pointer back to the center of the window
  //   wrap = true;
  //   glutWarpPointer(ww / 2, wh / 2);
  //   glutWarpPointer(0,0);
  //   windowX=glutGet(GLUT_WINDOW_X);
  //   windowY=glutGet(GLUT_WINDOW_Y);
  // } else {
  //   wrap = false;
  // }

  // static bool wrap = false;
  // if(!wrap){
  // windowX=glutGet(GLUT_WINDOW_X);
  // windowY=glutGet(GLUT_WINDOW_Y);
  // int toX=windowX+width/2,toY=windowY+height/2;
  // int dX=toX-x,dY=toY-y;
  // moveMouse(dX,dY);
  // }else{
  //   wrap=false;
  // }


  
}


static void keyboard(unsigned char key, int x, int y){
  unsigned char key_unshift=key;
  if('A'<=key_unshift&&key_unshift<='Z')key_unshift=key_unshift-'A'+'a';
  keystate[key_unshift]=true;
  keymodifiers=glutGetModifiers();
  switch (key) {
  case 'q':
  case 'Q':
    /* ESC �� q �� Q ���^�C�v������I�� */
    exit(0);
  case '\033':
    isPaused=!isPaused;
  default:
    break;
  }
}


static void keyboardUp(unsigned char key, int x, int y){
  unsigned char key_unshift=key;
  if('A'<=key_unshift&&key_unshift<='Z')key_unshift=key_unshift-'A'+'a';
  keystate[key_unshift]=false;
  keymodifiers=glutGetModifiers();
  switch (key) {
  case 'q':
  case 'Q':
    /* ESC �� q �� Q ���^�C�v������I�� */
    exit(0);
  case '\033':
  default:
    break;
  }
}


static void keySpecial(int key, int x, int y){
  keymodifiers=glutGetModifiers();
}



int main(int argc, char *argv[])
{
  if(argc>=2)fragshadername=argv[1];
  else fatal("input shader name");
  glutInitWindowSize(width,height);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow(argv[0]);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutPassiveMotionFunc(motion);
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyboardUp);
  glutSpecialFunc(keySpecial);
  glutSpecialUpFunc(keySpecial);
  glutTimerFunc(TIMER_INTERVAL,timer,0);
  init();
  sendResolution();
  sendCameraAng();
  sendCameraPos();
  
    glutSetCursor(GLUT_CURSOR_NONE); // GLUT_CURSOR_INHERIT

    /* These values are very device specific */
    int w = 1366;
    int h = 768;
    int d = 96;

    if (w < 1 || h < 1 || d < 1)
        fatal("Bad initial value(s).");

    mouse_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

    if (-1 == mouse_fd)
        fatal("open");

    printf("Initializing device screen map as %dx%d @ %ddpi\n", w, h, d);

    mouse_init(mouse_fd, w, h, d);

  glutMainLoop();
  return 0;
}
