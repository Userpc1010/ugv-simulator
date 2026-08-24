uniform highp mat4 u_projectionMatrix;
uniform highp mat4 u_viewMatrix;
uniform highp mat4 u_modelMatrix;

attribute highp vec4 a_position;
attribute highp vec2 a_textcoord;
attribute highp vec3 a_normal;

varying highp vec2 v_textcoord;

void main(void)
{
    mat4 viewmatrix = u_viewMatrix;
    viewmatrix[3][0] = 0.0f;
    viewmatrix[3][1] = 0.0f;
    viewmatrix[3][2] = 0.0f;
    mat4 mv_matrix = viewmatrix * u_modelMatrix;
    gl_Position = u_projectionMatrix * mv_matrix * a_position;
    v_textcoord = a_textcoord;
}
