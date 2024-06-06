function gradient_r_ee_in_phi = gradient_r_ee_in_phi_Func(Psi,L,d_o,lens_rot_angle,lens_angle,lens_fov)
%GRADIENT_R_EE_IN_PHI_FUNC
%    GRADIENT_R_EE_IN_PHI = GRADIENT_R_EE_IN_PHI_FUNC(IN1,IN2,D_O,LENS_ROT_ANGLE,LENS_ANGLE,LENS_FOV)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    30-Sep-2016 08:33:13

L1 = L(1);
L2 = L(2);
L3 = L(3);
delta1 = Psi(2);
delta2 = Psi(4);
delta3 = Psi(6);
qIns = Psi(7);
theta1l = Psi(1);
theta2l = Psi(3);
theta3l = Psi(5);
t2 = pi.*(1.0./2.0);
t3 = t2-theta1l;
t4 = cos(theta1l);
t5 = cos(theta2l);
t6 = sin(theta1l);
t7 = t2-theta3l;
t8 = 1.0./t7;
t9 = sin(delta1);
t10 = sin(delta2);
t11 = t10.^2;
t12 = sin(theta2l);
t13 = cos(delta1);
t14 = sin(theta3l);
t15 = t14-1.0;
t16 = cos(delta2);
t17 = delta2.*2.0;
t18 = sin(t17);
t19 = t12-1.0;
t20 = t2-theta2l;
t21 = 1.0./t20;
t22 = cos(theta3l);
t23 = 1.0./t3;
t24 = 1.0./t3.^2;
t25 = cos(delta3);
t26 = t13.^2;
t27 = t16.^2;
t28 = t12.*t27;
t29 = t11+t28;
t30 = t5.*t6.*t13.*t16;
t31 = delta1.*2.0;
t32 = sin(t31);
t33 = sin(delta3);
t34 = t11.*t12;
t35 = -t11+t34+1.0;
t36 = sin(lens_angle);
t37 = t6-1.0;
t51 = theta1l.*2.0;
t38 = pi-t51;
t42 = theta3l.*2.0;
t39 = pi-t42;
t40 = 1.0./t39;
t41 = t9.^2;
t43 = t5.*t6.*t9.*t10;
t44 = t4.*t18.*t19.*t32.*(1.0./4.0);
t47 = theta2l.*2.0;
t45 = pi-t47;
t46 = 1.0./t45;
t48 = lens_fov.*(1.0./2.0);
t49 = cos(lens_angle);
t50 = sin(lens_rot_angle);
t52 = 1.0./t38;
t53 = t6.*t41;
t54 = -t41+t53+1.0;
t55 = t4.*t5.*t9.*t10;
t56 = cos(lens_rot_angle);
t57 = t6.*t26;
t58 = t41+t57;
t59 = t18.*t19.*t32.*t37.*(1.0./4.0);
t60 = t4.*t5.*t13.*t16;
t61 = L1.*t6.*t23;
t62 = t4.*t12;
t63 = t30+t43+t62;
t64 = t6.*t9.*t35;
t65 = t6.*t13.*t18.*t19.*(1.0./2.0);
t66 = t64+t65-t4.*t5.*t10;
t67 = L3.*t8.*t15.*t33.*t66;
t68 = t6.*t13.*t29;
t69 = t6.*t9.*t18.*t19.*(1.0./2.0);
t70 = t68+t69-t4.*t5.*t16;
t71 = L3.*t8.*t15.*t25.*t70;
t72 = L2.*t6.*t13.*t16.*t19.*t21;
t73 = L2.*t6.*t9.*t10.*t19.*t21;
t74 = t61+t67+t71+t72+t73-L1.*t4.*t24-L2.*t4.*t5.*t21-L3.*t8.*t22.*t63;
t75 = L1.*t4.*t9.*t52.*2.0;
t76 = 1.0./t38.^2;
t77 = L1.*t9.*t37.*t76.*4.0;
t78 = t4.*t5.*t16.*t32.*(1.0./2.0);
t79 = t4.*t5.*t10.*t41;
t142 = t6.*t9.*t12;
t80 = t78+t79-t142;
t81 = t4.*t29.*t32.*(1.0./2.0);
t82 = t5.*t6.*t9.*t16;
t83 = t4.*t18.*t19.*t41.*(1.0./2.0);
t84 = t81+t82+t83;
t85 = L3.*t15.*t25.*t40.*t84.*2.0;
t86 = L2.*t5.*t6.*t9.*t46.*2.0;
t87 = t4.*t35.*t41;
t88 = t43+t44+t87;
t89 = L3.*t15.*t33.*t40.*t88.*2.0;
t90 = L2.*t4.*t16.*t19.*t32.*t46;
t91 = L2.*t4.*t10.*t19.*t41.*t46.*2.0;
t143 = L3.*t22.*t40.*t80.*2.0;
t92 = t75+t77+t85+t86+t89+t90+t91-t143;
t93 = t4.*t5.*t10.*t32.*(1.0./2.0);
t94 = t4.*t5.*t16.*t26;
t140 = t6.*t12.*t13;
t95 = t93+t94-t140;
t96 = L1.*t4.*t13.*t23;
t97 = L1.*t13.*t24.*t37;
t98 = t4.*t26.*t29;
t99 = t30+t44+t98;
t100 = L3.*t8.*t15.*t25.*t99;
t101 = L2.*t5.*t6.*t13.*t21;
t102 = t4.*t32.*t35.*(1.0./2.0);
t103 = t5.*t6.*t10.*t13;
t104 = t4.*t18.*t19.*t26.*(1.0./2.0);
t105 = t102+t103+t104;
t106 = L3.*t8.*t15.*t33.*t105;
t107 = L2.*t4.*t10.*t19.*t21.*t32.*(1.0./2.0);
t108 = L2.*t4.*t16.*t19.*t21.*t26;
t141 = L3.*t8.*t22.*t95;
t109 = t96+t97+t100+t101+t106+t107+t108-t141;
t110 = t5.*t16.*t58;
t111 = t4.*t12.*t13;
t112 = t5.*t10.*t32.*t37.*(1.0./2.0);
t113 = t110+t111+t112;
t114 = L1.*t13.*t23.*t37;
t115 = L2.*t16.*t19.*t21.*t58;
t116 = t32.*t35.*t37.*(1.0./2.0);
t117 = t18.*t19.*t58.*(1.0./2.0);
t145 = t4.*t5.*t10.*t13;
t118 = t116+t117-t145;
t119 = L3.*t8.*t15.*t33.*t118;
t120 = t29.*t58;
t121 = t59-t60+t120;
t122 = L3.*t8.*t15.*t25.*t121;
t123 = L2.*t10.*t19.*t21.*t32.*t37.*(1.0./2.0);
t144 = L3.*t8.*t22.*t113;
t146 = L2.*t4.*t5.*t13.*t21;
t124 = t114+t115+t119+t122+t123-t144-t146;
t125 = L1.*t9.*t37.*t52.*2.0;
t126 = t5.*t10.*t54;
t127 = t4.*t9.*t12;
t128 = t5.*t16.*t32.*t37.*(1.0./2.0);
t129 = t126+t127+t128;
t130 = L2.*t10.*t19.*t46.*t54.*2.0;
t131 = t35.*t54;
t132 = -t55+t59+t131;
t133 = L3.*t15.*t33.*t40.*t132.*2.0;
t134 = t18.*t19.*t54.*(1.0./2.0);
t135 = t29.*t32.*t37.*(1.0./2.0);
t150 = t4.*t5.*t9.*t16;
t136 = t134+t135-t150;
t137 = L3.*t15.*t25.*t40.*t136.*2.0;
t138 = L2.*t16.*t19.*t32.*t37.*t46;
t148 = L3.*t22.*t40.*t129.*2.0;
t149 = L2.*t4.*t5.*t9.*t46.*2.0;
t139 = -d_o+t125+t130+t133+t137+t138-t148-t149;
t147 = t50.*t124;
t151 = t56.*t139;
t152 = t147+t151;
t153 = L1.*t4.*t23;
t154 = L2.*t5.*t6.*t21;
t180 = t6.*t12;
t155 = t55+t60-t180;
t156 = t4.*t9.*t35;
t157 = t5.*t6.*t10;
t158 = t4.*t13.*t18.*t19.*(1.0./2.0);
t159 = t156+t157+t158;
t160 = L3.*t8.*t15.*t33.*t159;
t161 = t4.*t13.*t29;
t162 = t5.*t6.*t16;
t163 = t4.*t9.*t18.*t19.*(1.0./2.0);
t164 = t161+t162+t163;
t165 = L3.*t8.*t15.*t25.*t164;
t166 = L2.*t4.*t13.*t16.*t19.*t21;
t167 = L2.*t4.*t9.*t10.*t19.*t21;
t181 = L3.*t8.*t22.*t155;
t168 = qIns+t153+t154+t160+t165+t166+t167-t181;
t169 = t36.*t168;
t170 = t49.*t50.*t139;
t182 = t49.*t56.*t124;
t171 = t169+t170-t182;
t172 = cos(t48);
t173 = t9.*t13.*2.0;
t176 = t6.*t9.*t13.*2.0;
t174 = t173-t176;
t175 = cos(t31);
t177 = t18.*t19.*t37.*t175.*(1.0./2.0);
t178 = t18.*t19.*t174.*(1.0./2.0);
t179 = sin(t48);
t183 = t145-t150;
t184 = L3.*t8.*t22.*t183;
t185 = L3.*t8.*t15.*t33.*(t163-t4.*t13.*t35);
t186 = t158-t4.*t9.*t29;
t187 = L2.*t4.*t9.*t16.*t19.*t21;
t288 = L2.*t4.*t10.*t13.*t19.*t21;
t188 = t184+t185+t187-t288-L3.*t8.*t15.*t25.*t186;
t189 = t5.*t16.*t174;
t190 = t5.*t10.*t37.*t175;
t191 = -t127+t189+t190;
t192 = t29.*t174;
t193 = t150+t177+t192;
t194 = L3.*t8.*t15.*t25.*t193;
t195 = L2.*t16.*t19.*t21.*t174;
t196 = L2.*t4.*t5.*t9.*t21;
t197 = t35.*t37.*t175;
t198 = t55+t178+t197;
t199 = L3.*t8.*t15.*t33.*t198;
t200 = L2.*t10.*t19.*t21.*t37.*t175;
t213 = L1.*t9.*t23.*t37;
t214 = L3.*t8.*t22.*t191;
t201 = t194+t195+t196+t199+t200-t213-t214;
t202 = t5.*t16.*t37.*t175;
t215 = t5.*t10.*t174;
t203 = t111+t202-t215;
t204 = L3.*t22.*t40.*t203.*2.0;
t205 = L2.*t4.*t5.*t13.*t46.*2.0;
t206 = t35.*t174;
t207 = t145-t177+t206;
t208 = L3.*t15.*t33.*t40.*t207.*2.0;
t217 = t29.*t37.*t175;
t209 = t60+t178-t217;
t210 = L3.*t15.*t25.*t40.*t209.*2.0;
t211 = L2.*t10.*t19.*t46.*t174.*2.0;
t216 = L1.*t13.*t37.*t52.*2.0;
t218 = L2.*t16.*t19.*t37.*t46.*t175.*2.0;
t212 = t204+t205+t208+t210+t211-t216-t218;
t219 = t152.^2;
t220 = t171.^2;
t221 = t219+t220;
t222 = 1.0./sqrt(t221);
t223 = 1.0./t20.^2;
t224 = t4.*t12.*t13.*t16;
t225 = 1.0./t45.^2;
t226 = t4.*t9.*t10.*t12;
t227 = t5.*t18.*t32.*t37.*(1.0./4.0);
t228 = L2.*t5.*t6.*t223;
t229 = t5.*t6;
t230 = t224+t226+t229;
t231 = L3.*t8.*t22.*t230;
t232 = t4.*t5.*t9.*t18.*(1.0./2.0);
t233 = t4.*t5.*t13.*t27;
t234 = t232+t233-t6.*t12.*t16;
t235 = L3.*t8.*t15.*t25.*t234;
t236 = t4.*t5.*t13.*t18.*(1.0./2.0);
t237 = t4.*t5.*t9.*t11;
t238 = t236+t237-t6.*t10.*t12;
t239 = L3.*t8.*t15.*t33.*t238;
t240 = L2.*t4.*t5.*t13.*t16.*t21;
t241 = L2.*t4.*t5.*t9.*t10.*t21;
t242 = L2.*t4.*t13.*t16.*t19.*t223;
t243 = L2.*t4.*t9.*t10.*t19.*t223;
t244 = t228+t231+t235+t239+t240+t241+t242+t243-L2.*t6.*t12.*t21;
t245 = t12.*t16.*t58;
t246 = t10.*t12.*t32.*t37.*(1.0./2.0);
t281 = t4.*t5.*t13;
t247 = t245+t246-t281;
t248 = L3.*t8.*t22.*t247;
t249 = L2.*t16.*t19.*t58.*t223;
t250 = t5.*t27.*t58;
t251 = t224+t227+t250;
t252 = L3.*t8.*t15.*t25.*t251;
t253 = L2.*t4.*t12.*t13.*t21;
t254 = t5.*t18.*t58.*(1.0./2.0);
t255 = t4.*t10.*t12.*t13;
t256 = t5.*t11.*t32.*t37.*(1.0./2.0);
t257 = t254+t255+t256;
t258 = L3.*t8.*t15.*t33.*t257;
t259 = L2.*t5.*t16.*t21.*t58;
t260 = L2.*t5.*t10.*t21.*t32.*t37.*(1.0./2.0);
t261 = L2.*t10.*t19.*t32.*t37.*t223.*(1.0./2.0);
t282 = L2.*t4.*t5.*t13.*t223;
t262 = t248+t249+t252+t253+t258+t259+t260+t261-t282;
t263 = t10.*t12.*t54;
t264 = t12.*t16.*t32.*t37.*(1.0./2.0);
t283 = t4.*t5.*t9;
t265 = t263+t264-t283;
t266 = L3.*t22.*t40.*t265.*2.0;
t267 = L2.*t10.*t19.*t54.*t225.*4.0;
t268 = L2.*t4.*t9.*t12.*t46.*2.0;
t269 = t5.*t18.*t54.*(1.0./2.0);
t270 = t4.*t9.*t12.*t16;
t271 = t5.*t27.*t32.*t37.*(1.0./2.0);
t272 = t269+t270+t271;
t273 = L3.*t15.*t25.*t40.*t272.*2.0;
t274 = t5.*t11.*t54;
t275 = t226+t227+t274;
t276 = L3.*t15.*t33.*t40.*t275.*2.0;
t277 = L2.*t5.*t10.*t46.*t54.*2.0;
t278 = L2.*t16.*t19.*t32.*t37.*t225.*2.0;
t279 = L2.*t5.*t16.*t32.*t37.*t46;
t284 = L2.*t4.*t5.*t9.*t225.*4.0;
t280 = t266+t267+t268+t273+t276+t277+t278+t279-t284;
t285 = t10.*t16.*2.0;
t289 = t10.*t12.*t16.*2.0;
t286 = t285-t289;
t287 = cos(t17);
t290 = t32.*t37.*t286.*(1.0./2.0);
t291 = t19.*t32.*t37.*t287.*(1.0./2.0);
t292 = t4.*t13.*t286;
t293 = t4.*t9.*t19.*t287;
t294 = -t157+t292+t293;
t295 = L3.*t8.*t15.*t25.*t294;
t296 = t4.*t13.*t19.*t287;
t297 = t162+t296-t4.*t9.*t286;
t298 = L3.*t8.*t15.*t33.*t297;
t299 = t184+t187-t288+t295+t298;
t320 = t5.*t10.*t58;
t300 = t128-t320;
t301 = L3.*t8.*t22.*t300;
t302 = t58.*t286;
t303 = t145+t291+t302;
t322 = t19.*t58.*t287;
t304 = t60+t290-t322;
t305 = L3.*t8.*t15.*t33.*t304;
t306 = L2.*t10.*t19.*t21.*t58;
t321 = L3.*t8.*t15.*t25.*t303;
t323 = L2.*t16.*t19.*t21.*t32.*t37.*(1.0./2.0);
t307 = t301+t305+t306-t321-t323;
t317 = t5.*t16.*t54;
t308 = t112-t317;
t309 = L3.*t22.*t40.*t308.*2.0;
t310 = L2.*t16.*t19.*t46.*t54.*2.0;
t311 = t19.*t54.*t287;
t312 = t55+t290+t311;
t313 = L3.*t15.*t25.*t40.*t312.*2.0;
t314 = t54.*t286;
t315 = t150-t291+t314;
t318 = L3.*t15.*t33.*t40.*t315.*2.0;
t319 = L2.*t10.*t19.*t32.*t37.*t46;
t316 = t309+t310+t313-t318-t319;
t324 = 1.0./t7.^2;
t325 = 1.0./t39.^2;
t326 = L3.*t8.*t14.*t155;
t327 = L3.*t15.*t33.*t159.*t324;
t328 = L3.*t8.*t22.*t25.*t164;
t329 = L3.*t15.*t25.*t164.*t324;
t330 = L3.*t8.*t22.*t33.*t159;
t331 = t326+t327+t328+t329+t330-L3.*t22.*t155.*t324;
t332 = L3.*t14.*t40.*t129.*2.0;
t333 = L3.*t22.*t33.*t40.*t132.*2.0;
t334 = L3.*t15.*t33.*t132.*t325.*4.0;
t335 = L3.*t22.*t25.*t40.*t136.*2.0;
t336 = L3.*t15.*t25.*t136.*t325.*4.0;
t345 = L3.*t22.*t129.*t325.*4.0;
t337 = t332+t333+t334+t335+t336-t345;
t338 = L3.*t8.*t14.*t113;
t339 = L3.*t15.*t33.*t118.*t324;
t340 = L3.*t8.*t22.*t25.*t121;
t341 = L3.*t15.*t25.*t121.*t324;
t342 = L3.*t8.*t22.*t33.*t118;
t344 = L3.*t22.*t113.*t324;
t343 = t338+t339+t340+t341+t342-t344;
t346 = L3.*t8.*t15.*t25.*t159;
t347 = t346-L3.*t8.*t15.*t33.*t164;
t348 = L3.*t8.*t15.*t25.*t118;
t352 = L3.*t8.*t15.*t33.*t121;
t349 = t348-t352;
t350 = L3.*t15.*t25.*t40.*t132.*2.0;
t353 = L3.*t15.*t33.*t40.*t136.*2.0;
t351 = t350-t353;
gradient_r_ee_in_phi = [-t172.*(t49.*t74+t36.*t50.*t92-t36.*t56.*t109)+t179.*t222.*(t152.*(t56.*t92+t50.*t109).*2.0-t171.*(t36.*t74-t49.*t50.*t92+t49.*t56.*t109).*2.0).*(1.0./2.0);t172.*(-t49.*t188+t36.*t56.*t201+t36.*t50.*t212)+t179.*t222.*(t152.*(t50.*t201-t56.*t212).*2.0-t171.*(t36.*t188+t49.*t56.*t201+t49.*t50.*t212).*2.0).*(1.0./2.0);t172.*(t49.*t244+t36.*t56.*t262-t36.*t50.*t280)+t179.*t222.*(t152.*(t50.*t262+t56.*t280).*2.0+t171.*(t36.*t244-t49.*t56.*t262+t49.*t50.*t280).*2.0).*(1.0./2.0);-t172.*(-t49.*t299+t36.*t56.*t307+t36.*t50.*t316)-t179.*t222.*(t152.*(t50.*t307-t56.*t316).*2.0-t171.*(t36.*t299+t49.*t56.*t307+t49.*t50.*t316).*2.0).*(1.0./2.0);t172.*(t49.*t331-t36.*t50.*t337+t36.*t56.*t343)+t179.*t222.*(t152.*(t50.*t343+t56.*t337).*2.0+t171.*(t36.*t331+t49.*t50.*t337-t49.*t56.*t343).*2.0).*(1.0./2.0);t172.*(t49.*t347-t36.*t50.*t351+t36.*t56.*t349)+t179.*t222.*(t152.*(t50.*t349+t56.*t351).*2.0+t171.*(t36.*t347+t49.*t50.*t351-t49.*t56.*t349).*2.0).*(1.0./2.0);t49.*t172+t36.*t171.*t179.*t222;-t172.*(t36.*t50.*t124+t36.*t56.*t139)+t179.*t222.*(t171.*(t49.*t50.*t124+t49.*t56.*t139).*2.0+t152.*(t56.*t124-t50.*t139).*2.0).*(1.0./2.0)];
