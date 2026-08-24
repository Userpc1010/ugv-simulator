varying highp vec2 v_textcoord;
uniform sampler2D u_texture;

uniform int r_id;
uniform highp vec3 u_color;

void main(void)
{
    // r_id == 0: текстурирование (объекты, costmap)
    if (r_id == 0) {
        gl_FragColor = texture2D(u_texture, v_textcoord);
    }

    // r_id == 2: линии (uniform цвет)
    if (r_id == 2) {
        gl_FragColor = vec4(u_color, 1.0);
    }
}
