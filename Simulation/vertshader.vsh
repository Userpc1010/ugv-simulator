attribute highp vec4 a_position;
attribute highp vec2 a_textcoord;
attribute highp vec3 a_normal;

uniform highp mat4 u_projectionMatrix;
uniform highp mat4 u_viewMatrix;
uniform highp mat4 u_modelMatrix;

uniform int p_id;
uniform vec3 u_map_offset;

varying highp vec4 v_position;
varying highp vec2 v_textcoord;
varying highp vec3 v_normal;

void main(void)
{
    // p_id == 0: стандартное текстурирование (объекты, costmap)
    if (p_id == 0) {
        vec4 worldPos = u_modelMatrix * a_position;
        worldPos.xyz += u_map_offset;
        gl_Position = u_projectionMatrix * u_viewMatrix * worldPos;
        v_textcoord = a_textcoord;
    }

    // p_id == 2: линии
    if (p_id == 2) {
        vec4 worldPos = u_modelMatrix * a_position;
        worldPos.xyz += u_map_offset;
        gl_Position = u_projectionMatrix * u_viewMatrix * worldPos;
    }
}
