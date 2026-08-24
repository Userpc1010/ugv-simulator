varying highp vec2 v_textcoord;
uniform sampler2D u_texture;
void main(void)
{
    gl_FragColor = texture2D(u_texture, v_textcoord);
}
