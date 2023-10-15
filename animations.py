from manim import *

import math

# config.media_width = "75%"
config.verbosity = "WARNING"
#%%manim -qh CircleToSquare

# config.background_color = WHITE
# config.background_stroke_color = WHITE
# config.fill_color = BLACK

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

n = 100

COOL_WARM_RED = "#4AFAD9"
def invert_color(color):
    return [1.0 - color[0], 1.0 - color[1], 1.0 - color[2]]

def color_map(mix, colormap, customColor, value, darken = 1.0):
    # Get the color from the hsv colormap, which is a hsv colormap
    color = plt.get_cmap(colormap)(value)

    rgb = [1.0 - color[0], 1.0 - color[1], 1.0 - color[2]] 
    rgb[0] = (mix * rgb[0]) + ((1.0 - mix) * (1.0 - customColor[0] * darken))
    rgb[1] = (mix * rgb[1]) + ((1.0 - mix) * (1.0 - customColor[1] * darken))
    rgb[2] = (mix * rgb[2]) + ((1.0 - mix) * (1.0 - customColor[2] * darken))
    
    # Convert color to HEX form
    hex_color = mcolors.rgb2hex(rgb)
    return hex_color

def color_blend(mix, phi1, phi2, phi3, theta1, theta2, theta3):
    phiTotal = phi1 + phi2 + phi3
    if (phiTotal <= 0.0) : return mcolors.rgb2hex(invert_color([1.0, 1.0, 1.0]))

    rgb = [
        (phi1 * theta1[0] * theta1[0] + phi2 * theta2[0] * theta2[0] + phi3 * theta3[0] * theta3[0]) / phiTotal,
        (phi1 * theta1[1] * theta1[1] + phi2 * theta2[1] * theta2[1] + phi3 * theta3[1] * theta3[1]) / phiTotal,
        (phi1 * theta1[2] * theta1[2] + phi2 * theta2[2] * theta2[2] + phi3 * theta3[2] * theta3[2]) / phiTotal,
    ]
    rgb[0] = math.sqrt(rgb[0])
    rgb[1] = math.sqrt(rgb[1])
    rgb[2] = math.sqrt(rgb[2])

    rgb[0] = (mix * rgb[0]) + ((1.0 - mix) * (0.0))
    rgb[1] = (mix * rgb[1]) + ((1.0 - mix) * (0.0))
    rgb[2] = (mix * rgb[2]) + ((1.0 - mix) * (0.0))
    return mcolors.rgb2hex(invert_color(rgb))





def RBF(x, c, r, w):
    d = abs(x-c)
    return w * math.exp(-.5 * ((3*d)/r)**2)

def lerp(x, a, b):
    x = min(x, 1.0)
    x = max(x, 0.0)
    return (1.0 - x) * a + x * b

def PolarCurve(theta):
    r = 2 + 3 * np.cos(theta)
    vector = np.array([r * np.cos(theta), r * np.sin(theta), 0])
    return vector

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class Draw2DRBF(Scene):    
    def construct(self):

        radius = ValueTracker(3.0)
        weight = ValueTracker(1.0)
        r_offset = ValueTracker(0.0)
        theta_offset = ValueTracker(0.0)
        colorMapMode = ValueTracker(0.0)

        # formula = MathTex(r"\hat{\varphi}(d, r, w) = w * e^{-\frac{1}{2}{( \frac{3d}{r} )^2}}", color=BLACK)
        eq1 = MathTex(r"\varphi(r, w, d)  = w * e^{-\frac{1}{2}{(\frac{3d}{r})^2}}")
        self.play(Write(eq1), run_time=1)    
        self.wait()

        self.play(eq1.animate.shift(DOWN * 3))

        # self.play(Unwrite(eq1), run_time=1)    

        plot_2d = VGroup()

        polar_plane = PolarPlane(
            azimuth_units="PI radians",
            size=4,
            azimuth_label_font_size=33.6,
            radius_config={"font_size": 33.6},
            stroke_color=BLACK,
            color = BLACK,
            background_line_style={
                "stroke_color": WHITE,
                "stroke_width": 4,
                "stroke_opacity": 1.0
            }
        )

        plot_2d += polar_plane


        # self.camera.background_color = WHITE
        

        plot_1d = VGroup()
        ax_1d = Axes(
            x_range = [-5, 5, 1],
            y_range = [0, 2, .5],
            axis_config = {'include_numbers':False},
            x_axis_config={
                # "color": BLACK,
                # "decimal_number_config": {"color":"#010101"}
            },
            y_axis_config={
                # "color": BLACK,
                # "decimal_number_config": {"color":"#010101"}
            },
            x_length=6,
            y_length=4,
        )
        plot_1d += ax_1d

        # Text to display distribution mean
        radius_text = MathTex(r'r = ', color=COOL_WARM_RED).next_to(ax_1d, UP, buff=0.2)
        plot_1d += radius_text

        # Always redraw the decimal value for mu for each frame
        radius_value_text = always_redraw(
            lambda: DecimalNumber(num_decimal_places=2, color=COOL_WARM_RED)
            .set_value(radius.get_value())
            .next_to(radius_text, RIGHT, buff=0.2)
        )
        plot_1d += radius_value_text

        # curve = always_redraw(
        #     lambda: ax_1d.plot(lambda x: RBF(x, radius.get_value(), weight.get_value())).set_color_by_gradient(RED,YELLOW)
        # )

        curve = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-5, 5), lerp((i+1.5)/n,-5, 5)],  
                    [RBF(r_offset.get_value(), lerp(i/n,-5, 5), radius.get_value(), weight.get_value()), RBF(r_offset.get_value(), lerp((i+1.5)/n,-5, 5), radius.get_value(), weight.get_value())], 
                    line_color=color_map(colorMapMode.get_value(), "coolwarm", [0.0,0.0,0.0], RBF(r_offset.get_value(), lerp(i/n,-5, 5), radius.get_value(), weight.get_value())), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        plot_1d += curve

        


        

        # ax_2d = Axes(
        #     x_range = [-5, 5, 1],
        #     y_range = [-5, 5, 1],
        #     axis_config = {'include_numbers':False},
        #     x_axis_config={"color": BLACK},
        #     y_axis_config={"color": BLACK},
        #     x_length=4,
        #     y_length=4,
        # )

        circle_fill = always_redraw(lambda: VGroup(
           *[Circle(radius= ((n - i)/n) * (radius.get_value() / 2.0), color=color_map(colorMapMode.get_value(), "coolwarm", [0.0,0.0,0.0], i / n), fill_opacity=.05 * weight.get_value(), stroke_opacity=0.0).set_stroke(width=10) for i in range(n)], 
        ).shift(3 * RIGHT).shift(RIGHT * pol2cart(r_offset.get_value() * .5, theta_offset.get_value())[0] + UP * pol2cart(r_offset.get_value() * .5, theta_offset.get_value())[1]))
        circle_outline = always_redraw(lambda: DashedVMobject(
            Circle(color=COOL_WARM_RED, fill_opacity=0.0, stroke_opacity=1.0, radius=radius.get_value()/2.0).set_stroke(width=10)
        ).shift(3 * RIGHT).shift(RIGHT * pol2cart(r_offset.get_value() * .5, theta_offset.get_value())[0] + UP * pol2cart(r_offset.get_value() * .5, theta_offset.get_value())[1]))

        plot_1d.shift(3 * LEFT)

        plot_2d.shift(3 * RIGHT)
        
        self.play(Create(ax_1d), lag_ratio=.1)

        self.play(Create(curve),  run_time = 3, lag_ratio=.1)

        self.wait()
        self.play(Create(plot_2d), lag_ratio=.1)
        self.wait()
        self.play(FadeIn(circle_fill), run_time = 3, lag_ratio=.1)

        # Change the colormap to something nicer...
        self.play(
            colorMapMode.animate.set_value(1.0), run_time=3,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()

        # Show how radius 'r' effects things...
        r_line_1 = always_redraw(lambda: ax_1d.get_vertical_line(ax_1d.c2p(radius.get_value() + r_offset.get_value(), 1., 0), color=COOL_WARM_RED).set_stroke(width=10))
        r_line_2 = always_redraw(lambda: ax_1d.get_vertical_line(ax_1d.c2p(-radius.get_value() + r_offset.get_value(), 1., 0), color=COOL_WARM_RED).set_stroke(width=10))
        plot_1d += r_line_1
        plot_1d += r_line_2
        
        self.play(eq1[0][2].animate.set_color(COOL_WARM_RED), eq1[0][20].animate.set_color(COOL_WARM_RED), lag_ratio=.1)        
        self.play(Write(radius_text), Write(radius_value_text), Create(circle_outline), Create(r_line_1), Create(r_line_2), lag_ratio=.1)
        self.wait()
        self.play(
            radius.animate.set_value(1), run_time=1,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            radius.animate.set_value(4), run_time=1.5,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            radius.animate.set_value(3), run_time=2,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(Unwrite(radius_text), Unwrite(radius_value_text), Uncreate(circle_outline), Uncreate(r_line_1), Uncreate(r_line_2), lag_ratio=.1)
        self.play(eq1[0][2].animate.set_color(WHITE), eq1[0][20].animate.set_color(WHITE), lag_ratio=.1)
        
        # Show how weight 'w' effects things...
        w_line = always_redraw(lambda: Line(start = ax_1d.c2p(-5, weight.get_value(), 0), end = ax_1d.c2p(5.0, weight.get_value(), 0), color=COOL_WARM_RED).set_stroke(width=10))
        plot_1d += w_line
        weight_text = MathTex(r'w = ', color=COOL_WARM_RED).next_to(ax_1d, UP, buff=0.2)
        plot_1d += weight_text
        weight_value_text = always_redraw(
            lambda: DecimalNumber(num_decimal_places=2, color=COOL_WARM_RED)
            .set_value(weight.get_value())
            .next_to(weight_text, RIGHT, buff=0.2)
        )
        plot_1d += weight_value_text

        self.play(eq1[0][4].animate.set_color(COOL_WARM_RED), eq1[0][9].animate.set_color(COOL_WARM_RED), lag_ratio=.1)
        self.play(Write(weight_text), Write(weight_value_text), Create(w_line), lag_ratio=.1)
        self.wait()
        self.play(
            weight.animate.set_value(0), run_time=1,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            weight.animate.set_value(2), run_time=1.5,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            weight.animate.set_value(1), run_time=2,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(Unwrite(weight_text), Unwrite(weight_value_text), Uncreate(w_line), lag_ratio=.1)
        self.play(eq1[0][4].animate.set_color(WHITE), eq1[0][9].animate.set_color(WHITE), lag_ratio=.1)




        # Show how distance 'd' effects things...
        # eq2 = MathTex(r"\hat{\varphi}(r, w, {{ ||x - c|| }})  = w * e^{-\frac{1}{2}{( \frac{3d}{r} )^2}}").shift(DOWN*3)
        # self.play(TransformMatchingTex(eq1, eq2))
        distance_text = MathTex(r'd = ', color=COOL_WARM_RED).next_to(ax_1d, UP, buff=0.2)
        d_line_1 = always_redraw(lambda: Line(
            start=ax_1d.c2p(0.0, weight.get_value(), 0),
            end=ax_1d.c2p(r_offset.get_value(), weight.get_value(), 0), 
            color=COOL_WARM_RED
            ).set_stroke(width=10)
        )
        d_line_2 = always_redraw(lambda: Line(
            start=polar_plane.c2p(0,0,0),
            end=polar_plane.c2p(pol2cart(r_offset.get_value(), theta_offset.get_value())[0], pol2cart(r_offset.get_value(), theta_offset.get_value())[1], 0.0), 
            color=COOL_WARM_RED
            ).set_stroke(width=10)
        )
        plot_1d += d_line_1
        plot_1d += d_line_2

        plot_1d += distance_text
        distance_value_text = always_redraw(
            lambda: DecimalNumber(num_decimal_places=2, color=COOL_WARM_RED)
            .set_value(abs(r_offset.get_value()))
            .next_to(distance_text, RIGHT, buff=0.2)
        )
        plot_1d += distance_value_text
        self.play(eq1[0][6].animate.set_color(COOL_WARM_RED), eq1[0][18].animate.set_color(COOL_WARM_RED), lag_ratio=.1) # highlight d        
        self.play(Write(distance_text), Write(distance_value_text), Create(d_line_1), Create(d_line_2), lag_ratio=.1)

        self.play(
            r_offset.animate.set_value(-2.5), run_time=1,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            r_offset.animate.set_value(1.5), run_time=1.5,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()

        self.play(
            r_offset.animate.set_value(2.5),
            theta_offset.animate.set_value(.5*3.14), 
            run_time=2.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()

        self.play(
            r_offset.animate.set_value(0), 
            theta_offset.animate.set_value(0),
            run_time=1,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(Unwrite(distance_text), Unwrite(distance_value_text), Uncreate(d_line_1), Uncreate(d_line_2), lag_ratio=.1)
        self.play(eq1[0][6].animate.set_color(WHITE), eq1[0][18].animate.set_color(WHITE), lag_ratio=.1) #unhilight d
        self.wait()

        self.play(Uncreate(curve), Uncreate(circle_fill), lag_ratio=.1)
        self.play(Uncreate(ax_1d), Uncreate(polar_plane), lag_ratio=.1)
        self.play(eq1.animate.shift(UP * 3))
        self.wait()

class DrawMultiple(Scene):    
    def construct(self):
        color1 = plt.get_cmap("hsv")(0.0)
        color2 = plt.get_cmap("hsv")(0.4)
        color3 = plt.get_cmap("hsv")(0.6)

        color1 = [color1[0] * .999, color1[1] * .999, color1[2] * .999]
        color2 = [color2[0] * .5, color2[1] * .5, color2[2] * .5]
        color3 = [color3[0] * .999, color3[1] * .999, color3[2] * .999]

        color1inv = [1.0 - color1[0], 1.0 - color1[1], 1.0 - color1[2]]
        color2inv = [1.0 - color2[0], 1.0 - color2[1], 1.0 - color2[2]]
        color3inv = [1.0 - color3[0], 1.0 - color3[1], 1.0 - color3[2]]

        radius1 = ValueTracker(3.0)
        radius2 = ValueTracker(3.0)
        radius3 = ValueTracker(3.0)
        weight1 = ValueTracker(0.5)
        weight2 = ValueTracker(0.2)
        weight3 = ValueTracker(0.3)
        offset1 = ValueTracker(-1.5)
        offset2 = ValueTracker(1.0)
        offset3 = ValueTracker(-.5)
        colorMapMode = ValueTracker(1.0)

        eq1 = MathTex(r"\varphi({{r}}, {{w}},", "{{d}}",       r")", r" = w * e^{-\frac{1}{2}{(", r"\frac{3d}{r}", r")^2}}")
        eq2 = MathTex(r"\varphi({{r}}, {{w}},", "||x-{{c}}||", r")", r" = w * e^{-\frac{1}{2}{(", r"\frac{3||x-c||}{r}", r")^2}}")
        eq3 = MathTex(r"\varphi({{r}}, {{w}},", "||x-{{c}}||", r")")
        eq4 = MathTex(r"\varphi({{r_i}}, {{w_i}},", "||x-{{c_i}}||", r")")
        eq5 = MathTex(r"\sum_i^n", r"\varphi({{r_i}}, {{w_i}},", "||x-{{c_i}}||", r")")
        eq6 = MathTex(r"\varPhi(x) =", r"\sum_i^n", r"\varphi({{r_i}}, {{w_i}},", "||x-{{c_i}}||", r")")
        self.add(eq1)
        self.wait()
        self.play(TransformMatchingTex(eq1, eq2))
        self.wait()
        self.play(TransformMatchingTex(eq2, eq3))
        self.wait()
        self.play(TransformMatchingTex(eq3, eq4))
        self.wait()
        self.play(TransformMatchingTex(eq4, eq5))
        self.wait()
        self.play(TransformMatchingTex(eq5, eq6))
        self.wait()
        self.play(eq6.animate.shift(DOWN * 3))
        self.wait()

        plot_1d = VGroup()
        ax_1d = Axes(
            x_range = [-5, 5, 1],
            y_range = [0, 2, .5],
            axis_config = {'include_numbers':False},
            x_length=6,
            y_length=4,
        )
        plot_1d += ax_1d
        plot_1d.shift(3 * LEFT)
        self.play(Create(ax_1d), lag_ratio=.1)

        littlePhi1 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-5, 5), lerp((i+1.5)/n,-5, 5)],  
                    [RBF(offset1.get_value(), lerp(i/n,-5, 5), radius1.get_value(), weight1.get_value()), RBF(offset1.get_value(), lerp((i+1.5)/n,-5, 5), radius1.get_value(), weight1.get_value())], 
                    line_color=color_map(0.0, "hsv", [0.0, 0.0, 0.0], RBF(offset1.get_value(), lerp(i/n,-5, 5), radius1.get_value(), weight1.get_value()), darken=.999), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        littlePhi2 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-5, 5), lerp((i+1.5)/n,-5, 5)],  
                    [RBF(offset2.get_value(), lerp(i/n,-5, 5), radius2.get_value(), weight2.get_value()), RBF(offset2.get_value(), lerp((i+1.5)/n,-5, 5), radius2.get_value(), weight2.get_value())], 
                    line_color=color_map(0.0, "hsv", [0.0, 0.0, 0.0], RBF(offset2.get_value(), lerp(i/n,-5, 5), radius2.get_value(), weight2.get_value()), darken=.999), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        littlePhi3 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-5, 5), lerp((i+1.5)/n,-5, 5)],  
                    [RBF(offset3.get_value(), lerp(i/n,-5, 5), radius3.get_value(), weight3.get_value()), RBF(offset3.get_value(), lerp((i+1.5)/n,-5, 5), radius3.get_value(), weight3.get_value())], 
                    line_color=color_map(0.0, "hsv", [0.0, 0.0, 0.0], RBF(offset3.get_value(), lerp(i/n,-5, 5), radius3.get_value(), weight3.get_value()), darken=.999), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        littlePhi1Text = always_redraw(lambda: MathTex(r'\varphi_1(r = ' + "{:.1f}".format(radius1.get_value()) + r', w = ' + "{:.1f}".format(weight1.get_value()) + ', d = ' + "{:.1f}".format(abs(offset1.get_value())) + ')', color=WHITE).shift(3.5 * UP + 3.2 * LEFT))
        littlePhi2Text = always_redraw(lambda: MathTex(r'\varphi_2(r = ' + "{:.1f}".format(radius2.get_value()) + r', w = ' + "{:.1f}".format(weight2.get_value()) + ', d = ' + "{:.1f}".format(abs(offset2.get_value())) + ')', color=WHITE).shift(3.0 * UP + 3.2 * LEFT))
        littlePhi3Text = always_redraw(lambda: MathTex(r'\varphi_3(r = ' + "{:.1f}".format(radius3.get_value()) + r', w = ' + "{:.1f}".format(weight3.get_value()) + ', d = ' + "{:.1f}".format(abs(offset3.get_value())) + ')', color=WHITE).shift(2.5 * UP + 3.2 * LEFT))
        
        bigPhiText = always_redraw(lambda: VGroup(
                MathTex(r'\varPhi(0) = ' + "{:.1f}".format(RBF(offset1.get_value(), 0.0, radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), 0.0, radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), 0.0, radius3.get_value(), weight3.get_value())), color= color_map(1.0, "hsv", [0., 0., 0.], RBF(offset1.get_value(), 0.0, radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), 0.0, radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), 0.0, radius3.get_value(), weight3.get_value()), darken=.999)).shift(5 * LEFT), 
                Line(
                    3.75*LEFT,
                    ax_1d.c2p(0.0, RBF(offset1.get_value(), 0.0, radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), 0.0, radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), 0.0, radius3.get_value(), weight3.get_value()))
                )
            )                                
        )


        # distance_value_text = 
        #     lambda: DecimalNumber(num_decimal_places=2, color=COOL_WARM_RED)
        #     .set_value(abs(r_offset.get_value()))
        #     .next_to(distance_text, RIGHT, buff=0.2)
        # )
        

        bigPhi = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-5, 5), lerp((i+1.5)/n,-5, 5)],  
                    [RBF(offset1.get_value(), lerp(i/n,-5, 5), radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), lerp(i/n,-5, 5), radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), lerp(i/n,-5, 5), radius3.get_value(), weight3.get_value()), 
                     RBF(offset1.get_value(), lerp((i+1.5)/n,-5, 5), radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), lerp((i+1.5)/n,-5, 5), radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), lerp((i+1.5)/n,-5, 5), radius3.get_value(), weight3.get_value())], 
                    line_color=color_map(colorMapMode.get_value(), "hsv", [1.0,1.0,1.0],
                                         1.0 - (RBF(offset1.get_value(), lerp(i/n,-5, 5), radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), lerp(i/n,-5, 5), radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), lerp(i/n,-5, 5), radius3.get_value(), weight3.get_value()))), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )

        bigPhiAt0 = always_redraw(
            lambda: Dot(
                ax_1d.c2p(0.0, RBF(offset1.get_value(), 0.0, radius1.get_value(), weight1.get_value()) + RBF(offset2.get_value(), 0.0, radius2.get_value(), weight2.get_value()) + RBF(offset3.get_value(), 0.0, radius3.get_value(), weight3.get_value()))
            )
        )

        plot_1d += littlePhi1
        plot_1d += littlePhi2
        plot_1d += littlePhi3

        self.wait()
        self.play(Create(littlePhi1), Write(littlePhi1Text), lag_ratio=.2, run_time=1.5)
        self.play(Create(littlePhi2), Write(littlePhi2Text), lag_ratio=.2, run_time=1.5)
        self.play(Create(littlePhi3), Write(littlePhi3Text), lag_ratio=.2, run_time=1.5)
        self.wait()
        self.play(Create(bigPhi), Create(bigPhiAt0), Write(bigPhiText), lag_ratio=.1)
        self.wait()

        # Changing radius of splats
        self.play(
            radius1.animate.set_value(6.0),
            radius2.animate.set_value(6.0), 
            radius3.animate.set_value(6.0), 
            run_time=3.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            radius1.animate.set_value(0.5),
            radius2.animate.set_value(0.5), 
            radius3.animate.set_value(0.5), 
            run_time=1.5,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            radius1.animate.set_value(3.0),
            radius2.animate.set_value(3.0), 
            radius3.animate.set_value(3.0), 
            run_time=2.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()

        # Changing weights of splats
        self.play(
            weight1.animate.set_value(.9),
            weight2.animate.set_value(.1), 
            weight3.animate.set_value(.1), 
            run_time=1.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            weight1.animate.set_value(0.1),
            weight2.animate.set_value(0.9), 
            weight3.animate.set_value(0.1), 
            run_time=1.,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            weight1.animate.set_value(0.1),
            weight2.animate.set_value(0.1), 
            weight3.animate.set_value(0.9), 
            run_time=1.,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            weight1.animate.set_value(.5),
            weight2.animate.set_value(.2), 
            weight3.animate.set_value(.3), 
            run_time=2.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        
        # Changing offsets of splats
        self.play(
            offset1.animate.set_value(-5),
            offset2.animate.set_value(5), 
            offset3.animate.set_value(0), 
            run_time=2.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            offset1.animate.set_value(0),
            offset2.animate.set_value(0), 
            offset3.animate.set_value(0), 
            run_time=1.5,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()
        self.play(
            offset1.animate.set_value(-1.5),
            offset2.animate.set_value(1.0), 
            offset3.animate.set_value(-.5), 
            run_time=3.0,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait()

n=100

class DrawPolar(Scene):    
    def construct(self):
        plot_2d = VGroup()
        polar_plane = PolarPlane(
            azimuth_units="PI radians",
            size=4,
            azimuth_label_font_size=33.6,
            radius_config={"font_size": 33.6},
            stroke_color=BLACK,
            color = BLACK,
            background_line_style={
                "stroke_color": WHITE,
                "stroke_width": 4,
                "stroke_opacity": 1.0
            }
        )
        plot_2d += polar_plane
        plot_2d.shift(3 * LEFT + 1 * UP)

        self.play(Create(polar_plane))
        self.wait()

class DrawPolar2(Scene):    
    def construct(self):
        plot_2d = VGroup()
        polar_plane = PolarPlane(
            azimuth_units="PI radians",
            size=4,
            azimuth_label_font_size=33.6,
            radius_config={"font_size": 33.6},
            stroke_color=BLACK,
            color = BLACK,
            background_line_style={
                "stroke_color": WHITE,
                "stroke_width": 4,
                "stroke_opacity": 1.0
            }
        )
        plot_2d += polar_plane
        plot_2d.shift(3 * RIGHT)

        self.play(Create(polar_plane))
        self.wait()
        self.play(Uncreate(polar_plane))
        self.wait()




class DrawAttributes(Scene):    
    def construct(self):
        skip = 1.000

        # eq1 = MathTex(r"\varphi({{r}}, {{w}},", "{{d}}",       r")", r" = w * e^{-\frac{1}{2}{(", r"\frac{3d}{r}", r")^2}}")
        # eq2 = MathTex(r"\varphi({{r}}, {{w}},", "||x-{{c}}||", r")", r" = w * e^{-\frac{1}{2}{(", r"\frac{3||x-c||}{r}", r")^2}}")
        # eq3 = MathTex(r"\varphi({{r}}, {{w}},", "||x-{{c}}||", r")")
        # eq4 = MathTex(r"\varphi({{r_i}}, {{w_i}},", "||x-{{c_i}}||", r")")
        # eq5 = MathTex(r"\sum_i^n", r"\varphi({{r_i}}, {{w_i}},", "||x-{{c_i}}||", r")")
        eq1 = MathTex(r"\varphi_i \mapsto \in\mathbb{R}").shift(LEFT * 2)
        eq2 = MathTex(r"\varPhi =", r"\sum_i^n", r"\varphi_i \mapsto \in\mathbb{R}" ).shift(RIGHT * 2)

        eq3 = MathTex(r"\theta_i \mapsto ").shift(LEFT * 2 + DOWN * 1)
        eq4 = MathTex(r"\Theta = {\sum_i^n \theta_i * \varphi_i \over \varPhi} \mapsto ").shift(RIGHT * 2 + DOWN * 1)

        self.play(FadeIn(eq1), FadeIn(eq2), run_time = skip)
        self.wait(skip)

        self.play(eq1.animate.shift(UP), eq2.animate.shift(UP), run_time = skip)

        hue1 = ValueTracker(0.)
        hue2 = ValueTracker(0.)

        eq3_circle = always_redraw(
            lambda: Circle(color=color_map(1.0,"rainbow", [1.0, 1.0, 1.0], hue1.get_value(), 1.0), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq3)
        )
        self.play(Write(eq3), Create(eq3_circle), run_time = skip)
        self.play(hue1.animate.set_value(1.0), rate_func=rate_functions.smooth, run_time = 2.0 * skip)

        eq4_circle = always_redraw(
            lambda: Circle(color=color_map(1.0,"rainbow", [1.0, 1.0, 1.0], hue2.get_value(), 1.0), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq4)
        )
        self.wait(skip)
        self.play(Write(eq4), Create(eq4_circle), run_time = skip)
        self.play(hue2.animate.set_value(1.0), rate_func=rate_functions.smooth, run_time = 2.0 * skip)
        self.wait(skip)

        self.play(FadeOut(eq1), FadeOut(eq2), FadeOut(eq3), FadeOut(eq3_circle), FadeOut(eq4), FadeOut(eq4_circle), run_time = skip)
 

        radius = ValueTracker(1.5)
        weight = ValueTracker(0.3)
        distance1 = ValueTracker(3.0)
        distance2 = ValueTracker(3.0)
        distance3 = ValueTracker(3.0)

        fakeDistance1 = ValueTracker(-3.0)
        fakeDistance2 = ValueTracker(-6.0)
        fakeDistance3 = ValueTracker(-9.0)

        verticalOffset1 = ValueTracker(0.0)
        verticalOffset2 = ValueTracker(0.0)
        verticalOffset3 = ValueTracker(0.0)

        theta1 = ValueTracker(6.28 * 0.0 + 3.14 * .5)
        theta2 = ValueTracker(6.28 * .33 + 3.14 * .5)
        theta3 = ValueTracker(6.28 * .66 + 3.14 * .5)
        colorMapMode = ValueTracker(0.0)

        plot_2d = VGroup()
        polar_plane = PolarPlane(
            azimuth_units="PI radians",
            size=4,
            azimuth_label_font_size=33.6,
            radius_config={"font_size": 33.6},
            stroke_color=BLACK,
            color = BLACK,
            background_line_style={
                "stroke_color": WHITE,
                "stroke_width": 4,
                "stroke_opacity": 1.0
            }
        )
        plot_2d += polar_plane
        # circle_fill = always_redraw(lambda: VGroup(
        #    *[
        #        Circle(radius= ((n - i)/n) * (radius.get_value() / 2.0), color=color_map(colorMapMode.get_value(), "coolwarm", [1.0,0.0,0.0], i / n), fill_opacity=.1 * weight.get_value(), stroke_opacity=0.0).shift(3 * RIGHT).shift(RIGHT * pol2cart(distance1.get_value() * .5, theta1.get_value())[0] + UP * pol2cart(distance1.get_value() * .5, theta1.get_value())[1]).add(
        #        Circle(radius= ((n - i)/n) * (radius.get_value() / 2.0), color=color_map(colorMapMode.get_value(), "coolwarm", [0.0,0.0,1.0], i / n), fill_opacity=.1 * weight.get_value(), stroke_opacity=0.0).shift(3 * RIGHT).shift(RIGHT * pol2cart(distance2.get_value() * .5, theta2.get_value())[0] + UP * pol2cart(distance2.get_value() * .5, theta2.get_value())[1])).add(
        #        Circle(radius= ((n - i)/n) * (radius.get_value() / 2.0), color=color_map(colorMapMode.get_value(), "coolwarm", [0.0,1.0,0.0], i / n), fill_opacity=.1 * weight.get_value(), stroke_opacity=0.0).shift(3 * RIGHT).shift(RIGHT * pol2cart(distance3.get_value() * .5, theta3.get_value())[0] + UP * pol2cart(distance3.get_value() * .5, theta3.get_value())[1])) 
        #        for i in range(n)]
        #     )
        # )
        circle_outline_1 = always_redraw(lambda:
            Circle(color=mcolors.rgb2hex([1.0,1.0,0.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(polar_plane.c2p(0,0,0)).shift(RIGHT * pol2cart(distance1.get_value() * .5, theta1.get_value())[0] + UP * pol2cart(distance1.get_value() * .5, theta1.get_value())[1])
        )
        circle_outline_2 = always_redraw(lambda:
            Circle(color=mcolors.rgb2hex([0.0,1.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(polar_plane.c2p(0,0,0)).shift(RIGHT * pol2cart(distance2.get_value() * .5, theta2.get_value())[0] + UP * pol2cart(distance2.get_value() * .5, theta2.get_value())[1])
        )
        circle_outline_3 = always_redraw(lambda:
            Circle(color=mcolors.rgb2hex([1.0,0.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(polar_plane.c2p(0,0,0)).shift(RIGHT * pol2cart(distance3.get_value() * .5, theta3.get_value())[0] + UP * pol2cart(distance3.get_value() * .5, theta3.get_value())[1])
        )

        circle_outline_blended = always_redraw(lambda:
            Circle(color=color_blend(1.0,
                                        RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()),
                                        [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                    ), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(polar_plane.c2p(0,0,0))
        )

        # plot_2d += circle_fill
        plot_2d += circle_outline_1
        plot_2d += circle_outline_2
        plot_2d += circle_outline_3
        plot_2d += circle_outline_blended
        plot_2d.shift(3 * LEFT + 1 * UP)



        plot_1d = VGroup()
        ax_1d = Axes(
            x_range = [-11, 2, 12],
            y_range = [0, 1., .3],
            axis_config = {'include_numbers':False},
            # x_axis_config={
            #     "color": RED,
            #     "numbers_to_exclude": [2,3],
            #     "decimal_number_config": {
            #         "color": TEAL,
            #         "unit": "\\rm m",
            #         "num_decimal_places": 0
            #     }
            # },
            x_length=3,
            y_length=4,
        )
        plot_1d += ax_1d
        plot_1d.shift(3.5 * RIGHT + 1 * UP)
        ThetaEqOriginal = MathTex(r"\Theta = {", r"\sum_i^n \theta_i * \varphi_i \over \varPhi}").shift(DOWN * 2.4 + LEFT * 3)

        self.play(Write(ThetaEqOriginal), Create(ax_1d), run_time = skip)

        # # littlePhi1Text = always_redraw(lambda: MathTex(r'\varphi_1(0) = ' + "{:.1f}".format(RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value())), color=WHITE).shift(3.0 * UP + 5 * LEFT))
        # # littlePhi2Text = always_redraw(lambda: MathTex(r'\varphi_2(0) = ' + "{:.1f}".format(RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value())), color=WHITE).shift(3.0 * UP + 3 * LEFT))
        # # littlePhi3Text = always_redraw(lambda: MathTex(r'\varphi_3(0) = ' + "{:.1f}".format(RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value())), color=WHITE).shift(3.0 * UP + 1 * LEFT))
        # # # self.play(Write(littlePhi1Text), Write(littlePhi2Text), Write(littlePhi3Text)) #, polar_plane)

        # # littleTheta1Text = always_redraw(lambda: MathTex(r'\theta_1', color=rgb_to_hex([1.0,1.0,0.0])).shift(3.0 * UP + 3 * LEFT))
        # # littleTheta2Text = always_redraw(lambda: MathTex(r'\theta_2', color=rgb_to_hex([0.0,1.0,1.0])).shift(2.5 * UP + 3 * LEFT))
        # # littleTheta3Text = always_redraw(lambda: MathTex(r'\theta_3', color=rgb_to_hex([1.0,0.0,1.0])).shift(2.0 * UP + 3 * LEFT))
        # # # self.play(Write(littleTheta1Text), Write(littleTheta2Text), Write(littleTheta3Text)) #, polar_plane)

        littlePhi1 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-11, 2), lerp((i+1.5)/n,-11, 2)],  
                    [verticalOffset1.get_value() + RBF(fakeDistance1.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                     verticalOffset1.get_value() + RBF(fakeDistance1.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value())], 
                    line_color=rgb_to_hex([1.0,1.0,0.0]), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        littlePhi2 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-11, 2), lerp((i+1.5)/n,-11, 2)],  
                    [verticalOffset2.get_value() + RBF(fakeDistance2.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                     verticalOffset2.get_value() + RBF(fakeDistance2.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value())], 
                    line_color=rgb_to_hex([0.0,1.0,1.0]), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        littlePhi3 = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-11, 2), lerp((i+1.5)/n,-11, 2)],  
                    [verticalOffset3.get_value() + RBF(fakeDistance3.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                     verticalOffset3.get_value() + RBF(fakeDistance3.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value())], 
                    line_color=rgb_to_hex([1.0,0.0,1.0]), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )
        # self.add(littlePhi1, littlePhi2, littlePhi3)
        self.play(Create(littlePhi1), Create(littlePhi2), Create(littlePhi3), lag_ratio=.1, run_time = skip)
        self.wait(skip)

        # self.wait()
        # self.play(FadeIn(circle_fill), lag_ratio = .0)
        self.play(FadeIn(circle_outline_1), FadeIn(circle_outline_2), FadeIn(circle_outline_3), run_time = skip)
        self.wait(skip)

        # bigPhiText = always_redraw(lambda:
        #         MathTex(r'\varPhi(0) = ' + "{:.1f}".format(RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value())), 
        #                 color= color_map(colorMapMode.get_value(), "hsv", [0., 0., 0.], RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()), darken=.999)).shift(3.5 * UP + 5 * LEFT)   
        # )
        # # self.play(Write(bigPhiText))
        # # bigPhiAt0 = always_redraw(
        # #     lambda: Dot(ax_1d.c2p(0.0, RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()))).add(
        # #             Line(
        # #                 3.5 * UP + 3.5*LEFT,
        # #                 ax_1d.c2p(0.0, RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()))
        # #             ))
        # # )

        bigTheta = always_redraw(
            lambda: VGroup(
                *[ax_1d.plot_line_graph( 
                    [lerp(i/n,-11, 2), lerp((i+1.5)/n,-11, 2)],  
                    [RBF(fakeDistance1.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()) + RBF(fakeDistance2.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()) + RBF(fakeDistance3.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                     RBF(fakeDistance1.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value()) + RBF(fakeDistance2.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value()) + RBF(fakeDistance3.get_value(), lerp((i+1.5)/n,-11, 2), radius.get_value(), weight.get_value())], 
                    line_color=color_blend(
                                        colorMapMode.get_value(),
                                         RBF(fakeDistance1.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                                         RBF(fakeDistance2.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()), 
                                         RBF(fakeDistance3.get_value(), lerp(i/n,-11, 2), radius.get_value(), weight.get_value()),
                                         [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                         ), add_vertex_dots=False).set_stroke(width=10) for i in range(n)]
            )
        )

        self.play(Create(bigTheta), lag_ratio=.1, run_time = skip)
        self.play(Create(circle_outline_blended), run_time = skip)
        self.wait(skip)
        self.play(colorMapMode.animate.set_value(1.0), lag_ratio=.1, run_time = skip)
        self.wait(skip)

        # Rotating to illustrate ratios of colors with bars and lengths
        self.play(Rotate(plot_1d, angle=-PI/2), run_time = skip)
        self.wait(skip)

        self.play(
            distance1.animate.set_value(0.0),
            distance2.animate.set_value(0.0), 
            distance3.animate.set_value(0.0), 
            fakeDistance1.animate.set_value(0.0),
            fakeDistance2.animate.set_value(0.0), 
            fakeDistance3.animate.set_value(0.0), 
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.09
        )
        self.wait(skip)

        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.3), 
            verticalOffset3.animate.set_value(0.6), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.wait(skip)

        def generateThetaEq() : 
            eq = MathTex(r"\Theta = {", #r"\sum_i^n \theta_i * \varphi_i \over \varPhi} = ",
                    "{:.1f}".format(RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value())) + r" \theta_1 + " +
                    "{:.1f}".format(RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value())) + r" \theta_2 + " +
                    "{:.1f}".format(RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value())) + r" \theta_3" +
                    " \over " +
                    "{:.1f}".format(RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()) + RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value())) +
                    "} = ").shift(DOWN * 2.4 + LEFT * 2)
            
            eq[1][3].set_color(mcolors.rgb2hex([1.0,1.0,0.0]))
            eq[1][4].set_color(mcolors.rgb2hex([1.0,1.0,0.0]))
            eq[1][9].set_color(mcolors.rgb2hex([0.0,1.0,1.0]))
            eq[1][10].set_color(mcolors.rgb2hex([0.0,1.0,1.0]))
            eq[1][15].set_color(mcolors.rgb2hex([1.0,0.0,1.0]))
            eq[1][16].set_color(mcolors.rgb2hex([1.0,0.0,1.0]))
            return eq

        ThetaEq = always_redraw(lambda : generateThetaEq())

        eq1Div = MathTex(r"\div").shift(DOWN * 2.4 + RIGHT * 3.4)
        eq1MapsTo = MathTex(r"\mapsto").shift(DOWN * 2.4 + RIGHT * 5.5)
        self.play(TransformMatchingTex(ThetaEqOriginal, ThetaEq), FadeIn(eq1Div), FadeIn(eq1MapsTo), run_time = skip)

        self.wait()

        phi1LineNum = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineNum = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)
        phi3LineNum = Line(ax_1d.c2p(0, .6), ax_1d.c2p(0, .9), stroke_color=WHITE).set_stroke(width=10)

        phi1LineDen = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineDen = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)
        phi3LineDen = Line(ax_1d.c2p(0, .6), ax_1d.c2p(0, .9), stroke_color=WHITE).set_stroke(width=10)

        theta1Circle = Circle(color=mcolors.rgb2hex([1.0,1.0,0.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.0 + .3*.5))
        theta2Circle = Circle(color=mcolors.rgb2hex([0.0,1.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.3 + .3*.5))
        theta3Circle = Circle(color=mcolors.rgb2hex([1.0,0.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.6 + .3*.5))        
        ThetaCircle = Circle(color=color_blend(1.0,
                                        RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()),
                                        [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                    ), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq1MapsTo)

        self.play(Create(phi1LineDen), Create(phi2LineDen), Create(phi3LineDen), run_time = skip)
        self.play(phi1LineDen.animate.shift(DOWN*3), phi2LineDen.animate.shift(DOWN*3), phi3LineDen.animate.shift(DOWN*3), run_time = skip)
        
        self.play(Create(phi1LineNum), Create(phi2LineNum), Create(phi3LineNum), Create(theta1Circle), Create(theta2Circle), Create(theta3Circle), run_time = skip)
        self.play(phi1LineNum.animate.shift(DOWN*1.5), phi2LineNum.animate.shift(DOWN*1.5), phi3LineNum.animate.shift(DOWN*1.5),
                  theta1Circle.animate.shift(DOWN*1.5), theta2Circle.animate.shift(DOWN*1.5), theta3Circle.animate.shift(DOWN*1.5), run_time = skip)
        self.play(phi1LineNum.animate.set_color(mcolors.rgb2hex([1.0,1.0,0.0])), phi2LineNum.animate.set_color(mcolors.rgb2hex([0.0,1.0,1.0])), phi3LineNum.animate.set_color(mcolors.rgb2hex([1.0,0.0,1.0])), run_time = skip)
        self.play(FadeIn(ThetaCircle), run_time = skip)
        self.wait(skip)

        self.play(
            FadeOut(phi1LineDen), FadeOut(phi2LineDen), FadeOut(phi3LineDen), 
            FadeOut(phi1LineNum), FadeOut(phi2LineNum), FadeOut(phi3LineNum), 
            FadeOut(theta1Circle), FadeOut(theta2Circle), FadeOut(theta3Circle), 
            FadeOut(ThetaCircle), run_time = skip
        )
        self.wait(skip)

        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.play(
            distance1.animate.set_value(3.0),
            distance2.animate.set_value(3.0), 
            distance3.animate.set_value(3.0), 
            fakeDistance1.animate.set_value(-3.0),
            fakeDistance2.animate.set_value(-6.0), 
            fakeDistance3.animate.set_value(-9.0), 
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait(skip)

        self.play(
            distance1.animate.set_value(0.0),
            distance2.animate.set_value(3.0), 
            distance3.animate.set_value(0.0), 

            fakeDistance1.animate.set_value(0.0),
            fakeDistance2.animate.set_value(-6.0), 
            fakeDistance3.animate.set_value(0.0), 
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait(skip)

        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.3), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.play(eq1Div.animate.shift(LEFT * .64), eq1MapsTo.animate.shift(LEFT * .64), run_time = skip)

        phi1LineNum = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineNum = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)
        
        phi1LineDen = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineDen = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)

        theta1Circle = Circle(color=mcolors.rgb2hex([1.0,1.0,0.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.0 + .3*.5))
        theta2Circle = Circle(color=mcolors.rgb2hex([1.0,0.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.3 + .3*.5))
        ThetaCircle = Circle(color=color_blend(1.0,
                                        RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()),
                                        [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                    ), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq1MapsTo)

        self.play(Create(phi1LineDen), Create(phi2LineDen), run_time = skip)
        self.play(phi1LineDen.animate.shift(DOWN*3), phi2LineDen.animate.shift(DOWN*3), run_time = skip)
        self.play(Create(phi1LineNum), Create(phi2LineNum), Create(theta1Circle), Create(theta2Circle),
                  run_time = skip)
        self.play(phi1LineNum.animate.shift(DOWN*1.5), phi2LineNum.animate.shift(DOWN*1.5),
                  theta1Circle.animate.shift(DOWN*1.5), theta2Circle.animate.shift(DOWN*1.5),
                  run_time = skip)

        self.play(phi1LineNum.animate.set_color(mcolors.rgb2hex([1.0,1.0,0.0])), phi2LineNum.animate.set_color(mcolors.rgb2hex([1.0,0.0,1.0])), run_time = skip)
        self.play(FadeIn(ThetaCircle), run_time = skip)
        self.wait(skip)

        self.play(
            FadeOut(phi1LineDen), FadeOut(phi2LineDen),
            FadeOut(phi1LineNum), FadeOut(phi2LineNum),
            FadeOut(theta1Circle), FadeOut(theta2Circle),
            FadeOut(ThetaCircle), run_time = skip
        )
        self.wait(skip)


        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.play(
            distance1.animate.set_value(3.0),
            distance2.animate.set_value(0.0), 
            distance3.animate.set_value(0.0), 

            fakeDistance1.animate.set_value(-3.0),
            fakeDistance2.animate.set_value(0.0), 
            fakeDistance3.animate.set_value(0.0),
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait(skip)

        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.3), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.wait(skip)

        phi1LineNum = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineNum = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)
        
        phi1LineDen = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineDen = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)

        theta1Circle = Circle(color=mcolors.rgb2hex([1.0,0.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.0 + .3*.5))
        theta2Circle = Circle(color=mcolors.rgb2hex([0.0,1.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.3 + .3*.5))
        ThetaCircle = Circle(color=color_blend(1.0,
                                        RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()),
                                        [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                    ), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq1MapsTo)

        self.play(Create(phi1LineDen), Create(phi2LineDen), run_time = skip)
        self.play(phi1LineDen.animate.shift(DOWN*3), phi2LineDen.animate.shift(DOWN*3), run_time = skip)
        self.play(Create(phi1LineNum), Create(phi2LineNum), Create(theta1Circle), Create(theta2Circle), run_time = skip)
        self.play(phi1LineNum.animate.shift(DOWN*1.5), phi2LineNum.animate.shift(DOWN*1.5),
                  theta1Circle.animate.shift(DOWN*1.5), theta2Circle.animate.shift(DOWN*1.5), run_time = skip)

        self.play(phi1LineNum.animate.set_color(mcolors.rgb2hex([1.0,0.0,1.0])), phi2LineNum.animate.set_color(mcolors.rgb2hex([0.0,1.0,1.0])), run_time = skip)
        self.play(FadeIn(ThetaCircle), run_time = skip)
        self.wait(skip)

        self.play(
            FadeOut(phi1LineDen), FadeOut(phi2LineDen),
            FadeOut(phi1LineNum), FadeOut(phi2LineNum),
            FadeOut(theta1Circle), FadeOut(theta2Circle),
            FadeOut(ThetaCircle), run_time = skip
        )

        self.wait(skip)


        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.play(
            distance1.animate.set_value(0.0),
            distance2.animate.set_value(0.0), 
            distance3.animate.set_value(3.0), 

            fakeDistance1.animate.set_value(0.0),
            fakeDistance2.animate.set_value(0.0), 
            fakeDistance3.animate.set_value(-9.0),
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait(skip)


        self.play(
            verticalOffset1.animate.set_value(0.3),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        phi1LineNum = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineNum = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)
        
        phi1LineDen = Line(ax_1d.c2p(0, 0), ax_1d.c2p(0, .3), stroke_color=WHITE).set_stroke(width=10)
        phi2LineDen = Line(ax_1d.c2p(0, .3), ax_1d.c2p(0, .6), stroke_color=WHITE).set_stroke(width=10)

        theta1Circle = Circle(color=mcolors.rgb2hex([0.0,1.0,1.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.0 + .3*.5))
        theta2Circle = Circle(color=mcolors.rgb2hex([1.0,1.0,0.0]), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).shift(ax_1d.c2p(0,.3 + .3*.5))
        ThetaCircle = Circle(color=color_blend(1.0,
                                        RBF(distance1.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance2.get_value(), 0.0, radius.get_value(), weight.get_value()), 
                                        RBF(distance3.get_value(), 0.0, radius.get_value(), weight.get_value()),
                                        [0.0,0.0,1.0], [1.0,0.0,0.0], [0.0,1.0,0.0]
                                    ), stroke_color=WHITE, fill_opacity=1.0, stroke_opacity=1.0, radius=.2).next_to(eq1MapsTo)

        self.play(Create(phi1LineDen), Create(phi2LineDen), run_time = skip)
        self.play(phi1LineDen.animate.shift(DOWN*3), phi2LineDen.animate.shift(DOWN*3), run_time = skip)
        self.play(Create(phi1LineNum), Create(phi2LineNum), Create(theta1Circle), Create(theta2Circle), run_time = skip)
        self.play(phi1LineNum.animate.shift(DOWN*1.5), phi2LineNum.animate.shift(DOWN*1.5),
                  theta1Circle.animate.shift(DOWN*1.5), theta2Circle.animate.shift(DOWN*1.5), run_time = skip)

        self.play(phi1LineNum.animate.set_color(mcolors.rgb2hex([0.0,1.0,1.0])), phi2LineNum.animate.set_color(mcolors.rgb2hex([1.0,1.0,0.0])), run_time = skip)
        self.play(FadeIn(ThetaCircle), run_time = skip)
        self.wait(skip)

        self.play(
            FadeOut(ThetaEq), FadeOut(eq1Div), FadeOut(eq1MapsTo), 
            FadeOut(phi1LineDen), FadeOut(phi2LineDen),
            FadeOut(phi1LineNum), FadeOut(phi2LineNum),
            FadeOut(theta1Circle), FadeOut(theta2Circle),
            FadeOut(ThetaCircle), run_time = skip
        )

        self.wait(skip)

        self.play(
            verticalOffset1.animate.set_value(0.0),
            verticalOffset2.animate.set_value(0.0), 
            verticalOffset3.animate.set_value(0.0), 
            run_time=1.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )

        self.play(
            distance1.animate.set_value(3.0),
            distance2.animate.set_value(3.0), 
            distance3.animate.set_value(3.0), 

            fakeDistance1.animate.set_value(-3.0),
            fakeDistance2.animate.set_value(-6.0), 
            fakeDistance3.animate.set_value(-9.0),
            run_time=2.0 * skip,
            rate_func=rate_functions.smooth, lag_ratio=.1
        )
        self.wait(skip)