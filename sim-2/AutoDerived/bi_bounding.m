function bi = bi_bounding(in1,in2)
%BI_BOUNDING
%    BI = BI_BOUNDING(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    12-Nov-2013 15:58:08

I11 = in2(10,:);
I12 = in2(11,:);
I21 = in2(12,:);
I22 = in2(13,:);
a1 = in1(7,:);
a2 = in1(9,:);
c11 = in2(2,:);
c12 = in2(3,:);
c21 = in2(4,:);
c22 = in2(5,:);
da1 = in1(8,:);
da2 = in1(10,:);
dphi = in1(12,:);
dth = in1(6,:);
dx = in1(2,:);
dy = in1(4,:);
ll = in2(1,:);
m11 = in2(6,:);
m12 = in2(7,:);
m21 = in2(8,:);
m22 = in2(9,:);
ma1 = in2(19,:);
ma2 = in2(20,:);
mmh = in2(21,:);
mms = in2(22,:);
ms = in2(16,:);
mtd = in2(18,:);
phi = in1(11,:);
sep = in2(17,:);
slength = in2(14,:);
th = in1(5,:);
x = in1(1,:);
y = in1(3,:);
t2 = 1.0./sep;
t5 = mtd.*phi.*t2;
t3 = a2-ma2-t5-th;
t4 = cos(t3);
t6 = a2+ma2+t5+th;
t7 = cos(t6);
t8 = ll.*t7.*2.0;
t9 = a1-ma1-th;
t10 = cos(t9);
t11 = ll.*t10.*2.0;
t12 = a1+ma1+th;
t13 = cos(t12);
t15 = ll.*t13.*2.0;
t14 = t11-t15;
t16 = mtd.*phi.*t2.*(1.0./7.0);
t17 = t16+th;
t18 = sin(t17);
t19 = slength.*t18.*(1.0./4.0);
t20 = mtd.*phi.*t2.*(2.0./7.0);
t21 = t20+th;
t22 = sin(t21);
t23 = slength.*t22.*(1.0./4.0);
t24 = mtd.*phi.*t2.*(3.0./7.0);
t25 = t24+th;
t26 = sin(t25);
t27 = slength.*t26.*(1.0./4.0);
t28 = t5+th;
t29 = sin(t28);
t30 = slength.*t29.*(1.0./4.0);
t31 = mtd.*phi.*t2.*(4.0./7.0);
t32 = t31+th;
t33 = sin(t32);
t34 = slength.*t33.*(1.0./4.0);
t35 = mtd.*phi.*t2.*(5.0./7.0);
t36 = t35+th;
t37 = sin(t36);
t38 = slength.*t37.*(1.0./4.0);
t39 = mtd.*phi.*t2.*(6.0./7.0);
t40 = t39+th;
t41 = sin(t40);
t42 = slength.*t41.*(1.0./4.0);
t43 = c21.*t4.*2.0;
t44 = c11.*m11.*t10;
t45 = mtd.*slength.*t2.*t29.*(1.0./4.0);
t46 = mtd.*slength.*t2.*t18.*(1.0./2.8e1);
t47 = mtd.*slength.*t2.*t22.*(1.0./1.4e1);
t48 = mtd.*slength.*t2.*t26.*(3.0./2.8e1);
t49 = mtd.*slength.*t2.*t33.*(1.0./7.0);
t50 = mtd.*slength.*t2.*t37.*(5.0./2.8e1);
t51 = mtd.*slength.*t2.*t41.*(3.0./1.4e1);
t52 = sin(t6);
t53 = cos(t28);
t54 = slength.*t53.*(1.0./4.0);
t55 = cos(t17);
t56 = slength.*t55.*(1.0./4.0);
t57 = cos(t21);
t58 = slength.*t57.*(1.0./4.0);
t59 = cos(t25);
t60 = slength.*t59.*(1.0./4.0);
t61 = cos(t32);
t62 = slength.*t61.*(1.0./4.0);
t63 = cos(t36);
t64 = slength.*t63.*(1.0./4.0);
t65 = cos(t40);
t66 = slength.*t65.*(1.0./4.0);
t67 = sin(t9);
t68 = ll.*t67.*2.0;
t69 = sin(t12);
t70 = sin(t3);
t71 = ll.*t52.*2.0;
t72 = ll.*t69.*2.0;
t73 = t68+t72;
t74 = c11.*m11.*t67;
t75 = mtd.*slength.*t2.*t55.*(1.0./2.8e1);
t76 = mtd.*slength.*t2.*t57.*(1.0./1.4e1);
t77 = mtd.*slength.*t2.*t59.*(3.0./2.8e1);
t78 = mtd.*slength.*t2.*t53.*(1.0./4.0);
t79 = mtd.*slength.*t2.*t61.*(1.0./7.0);
t80 = mtd.*slength.*t2.*t63.*(5.0./2.8e1);
t81 = mtd.*slength.*t2.*t65.*(3.0./1.4e1);
t82 = ll.*mtd.*t2.*t52.*2.0;
t83 = mmh.*2.0;
t84 = m11+m12+m21+m22+mms+ms+t83;
t85 = c21.*t70.*2.0;
t86 = slength.*t53.*(1.0./3.2e1);
t87 = slength.*t55.*(7.0./3.2e1);
t88 = slength.*t57.*(3.0./1.6e1);
t89 = slength.*t59.*(5.0./3.2e1);
t90 = slength.*t61.*(1.0./8.0);
t91 = slength.*t63.*(3.0./3.2e1);
t92 = slength.*t65.*(1.0./1.6e1);
t93 = c12.*t69.*2.0;
t94 = ll.*t70.*2.0;
t95 = mtd.*slength.*t2.*t18.*(1.0./5.6e1);
t96 = mtd.*slength.*t2.*t22.*(1.0./2.8e1);
t97 = mtd.*slength.*t2.*t26.*(3.0./5.6e1);
t98 = mtd.*slength.*t2.*t33.*(1.0./1.4e1);
t99 = slength.*t18.*(1.0./8.0);
t100 = slength.*t22.*(1.0./8.0);
t101 = slength.*t26.*(1.0./8.0);
t102 = slength.*t33.*(1.0./8.0);
t103 = ll.*t10;
t104 = ll.*t13;
t105 = mtd.*slength.*t2.*t55.*(1.0./5.6e1);
t106 = mtd.*slength.*t2.*t57.*(1.0./2.8e1);
t107 = mtd.*slength.*t2.*t59.*(3.0./5.6e1);
t108 = mtd.*slength.*t2.*t61.*(1.0./1.4e1);
t109 = slength.*t55.*(1.0./8.0);
t110 = slength.*t57.*(1.0./8.0);
t111 = slength.*t59.*(1.0./8.0);
t112 = ll.*t67;
t113 = mtd.*slength.*t2.*t53.*(1.0./8.0);
t114 = mtd.*slength.*t2.*t63.*(5.0./5.6e1);
t115 = mtd.*slength.*t2.*t65.*(3.0./2.8e1);
t116 = slength.*t53.*(1.0./8.0);
t117 = slength.*t63.*(1.0./8.0);
t118 = slength.*t65.*(1.0./8.0);
t119 = mtd.*slength.*t2.*t29.*(1.0./8.0);
t120 = mtd.*slength.*t2.*t37.*(5.0./5.6e1);
t121 = mtd.*slength.*t2.*t41.*(3.0./2.8e1);
t122 = slength.*t29.*(1.0./8.0);
t123 = slength.*t37.*(1.0./8.0);
t124 = slength.*t41.*(1.0./8.0);
t125 = ll.*mtd.*t2.*t52;
t126 = ll.*t52;
t127 = slength.*t29.*(1.0./6.4e1);
t128 = slength.*t18.*(7.0./6.4e1);
t129 = slength.*t22.*(3.0./3.2e1);
t130 = slength.*t26.*(5.0./6.4e1);
t131 = slength.*t33.*(1.0./1.6e1);
t132 = slength.*t37.*(3.0./6.4e1);
t133 = slength.*t41.*(1.0./3.2e1);
t135 = ll.*t69;
t134 = t103+t104-t112+t127+t128+t129+t130+t131+t132+t133+t135-x+y;
t136 = t103+t104-t109-t112+t127+t128+t129+t130+t131+t132+t133+t135-x+y;
t137 = t103+t104-t109-t110-t112+t127+t128+t129+t130+t131+t132+t133+t135-x+y;
t138 = slength.*t53.*(1.0./6.4e1);
t139 = slength.*t55.*(7.0./6.4e1);
t140 = slength.*t57.*(3.0./3.2e1);
t141 = slength.*t59.*(5.0./6.4e1);
t142 = slength.*t61.*(1.0./1.6e1);
t143 = slength.*t63.*(3.0./6.4e1);
t144 = slength.*t65.*(1.0./3.2e1);
t145 = t112-t135+t138+t139+t140+t141+t142+t143+t144+x;
t147 = slength.*t55.*(1.0./6.4e1);
t146 = t138+t141+t142+t143+t144-t147-slength.*t57.*(1.0./3.2e1);
t148 = t138+t140+t141+t142+t143+t144-t147;
t149 = t138+t139+t140+t141+t142+t143+t144;
t150 = t103+t104+y;
t151 = -t103-t104+t112-t135+t138+t139+t140+t141+t142+t143+t144+x-y;
t152 = t112-t135+x;
t153 = t103+t104+t127+t128+t129+t130+t131+t132+t133+y;
t154 = t99+t103+t104+y;
t155 = t109+t112-t135+x;
t156 = -t99-t103-t104+t112-t135+t138+t139+t140+t141+t142+t143+t144+x-y;
t158 = slength.*t18.*(1.0./6.4e1);
t157 = t127+t130+t131+t132+t133-t158-slength.*t22.*(1.0./3.2e1);
t159 = t127+t129+t130+t131+t132+t133-t158;
t160 = t127+t128+t129+t130+t131+t132+t133;
t161 = t109+t110+t112-t135+x;
t162 = t99+t100+t103+t104+y;
t163 = -t99-t100-t103-t104+t112-t135+t138+t139+t140+t141+t142+t143+t144+x-y;
t164 = t109+t110+t111+t112-t135;
t165 = t99+t100+t101+t103+t104;
t166 = t99+t100+t101+t102+t103+t104+t122+t123+t124;
t167 = t90+t109+t110+t111+t112+t116+t117+t118-t135;
t168 = t103+t104+t127+t128+t129+t130+t131+t132+t133;
t169 = t112-t135+t138+t139+t140+t141+t142+t143+t144;
t184 = c12.*t13;
t170 = t103+t184;
t183 = c12.*t69;
t171 = t112-t183;
t172 = t103+t104;
t173 = t112-t135;
t174 = c21.*t70;
t187 = ll.*t70;
t175 = t90+t109+t110+t111+t112+t116+t117+t118+t126-t135+t174-t187;
t176 = c21.*t4;
t179 = ll.*t7;
t188 = ll.*t4;
t177 = t99+t100+t101+t102+t103+t104+t122+t123+t124+t176-t179-t188;
t178 = c22.*t7;
t180 = t99+t100+t101+t102+t103+t104+t122+t123+t124+t178-t179;
t189 = c22.*t52;
t181 = t90+t109+t110+t111+t112+t116+t117+t118+t126-t135-t189;
t182 = c11.^2;
t185 = t103-t104;
t186 = t112+t135;
t190 = t10.^2;
t191 = t182.*t190.*2.0;
t192 = t67.^2;
t193 = t182.*t192.*2.0;
t194 = t191+t193;
t195 = m11.*t194.*(1.0./2.0);
t196 = t11+t15+t19+t23+t27;
t197 = mms.*t196.*(1.0./2.0);
t198 = c22.*t7.*2.0;
t199 = -t8+t11+t15+t19+t23+t27+t30+t34+t38+t42+t198;
t200 = m22.*t199.*(1.0./2.0);
t201 = c12.*t13.*2.0;
t202 = t11+t201;
t203 = m12.*t202.*(1.0./2.0);
t204 = t11+t15;
t205 = mmh.*t204.*(1.0./2.0);
t206 = t11+t15+t19+t23+t27+t30+t34+t38+t42;
t207 = mmh.*t206.*(1.0./2.0);
t208 = slength.*t29.*(1.0./3.2e1);
t209 = slength.*t18.*(7.0./3.2e1);
t210 = slength.*t22.*(3.0./1.6e1);
t211 = slength.*t26.*(5.0./3.2e1);
t212 = slength.*t37.*(3.0./3.2e1);
t213 = slength.*t41.*(1.0./1.6e1);
t214 = t11+t15+t102+t208+t209+t210+t211+t212+t213;
t215 = ms.*t214.*(1.0./2.0);
t302 = ll.*t4.*2.0;
t216 = -t8+t11+t15+t19+t23+t27+t30+t34+t38+t42+t43-t302;
t217 = m21.*t216.*(1.0./2.0);
t218 = t44+t197+t200+t203+t205+t207+t215+t217;
t219 = t68+t93;
t220 = m12.*t219.*(1.0./2.0);
t221 = m21.*t73.*(1.0./2.0);
t222 = m22.*t73.*(1.0./2.0);
t223 = mmh.*t73;
t224 = mms.*t73.*(1.0./2.0);
t225 = ms.*t73.*(1.0./2.0);
t226 = t74+t220+t221+t222+t223+t224+t225;
t227 = t103-t184;
t228 = t112+t183;
t229 = t185.^2;
t230 = t229.*2.0;
t231 = t186.^2;
t232 = t231.*2.0;
t233 = t230+t232;
t234 = t171.*t228.*2.0;
t235 = t170.*t227.*2.0;
t236 = t234+t235;
t237 = m12.*t236.*(1.0./2.0);
t238 = t166.*t185.*2.0;
t239 = t167.*t186.*2.0;
t240 = t238+t239;
t241 = mmh.*t240.*(1.0./2.0);
t242 = t168.*t185.*2.0;
t243 = t169.*t186.*2.0;
t244 = t242+t243;
t245 = ms.*t244.*(1.0./2.0);
t246 = t173.*t186.*2.0;
t247 = t172.*t185.*2.0;
t248 = t246+t247;
t249 = mmh.*t248.*(1.0./2.0);
t250 = t175.*t186.*2.0;
t251 = t177.*t185.*2.0;
t252 = t250+t251;
t253 = m21.*t252.*(1.0./2.0);
t254 = t181.*t186.*2.0;
t255 = t180.*t185.*2.0;
t256 = t254+t255;
t257 = m22.*t256.*(1.0./2.0);
t258 = t165.*t185.*2.0;
t259 = t164.*t186.*2.0;
t260 = t258+t259;
t261 = mms.*t260.*(1.0./2.0);
t262 = t195+t237+t241+t245+t249+t253+t257+t261;
t263 = m21.*t14.*(1.0./2.0);
t264 = m22.*t14.*(1.0./2.0);
t265 = mmh.*t14;
t266 = mms.*t14.*(1.0./2.0);
t267 = ms.*t14.*(1.0./2.0);
t268 = t176+t179-t188;
t269 = t126-t174+t187;
t270 = t178-t179;
t271 = t126-t189;
t301 = c22.*mtd.*t2.*t52;
t272 = t105+t106+t107+t108+t113+t114+t115+t125-t301;
t273 = c22.*mtd.*t2.*t7;
t277 = ll.*mtd.*t2.*t7;
t274 = t95+t96+t97+t98+t119+t120+t121+t273-t277;
t275 = c21.*mtd.*t2.*t70;
t299 = ll.*mtd.*t2.*t70;
t276 = t105+t106+t107+t108+t113+t114+t115+t125+t275-t299;
t278 = c21.*mtd.*t2.*t4;
t279 = t105+t106+t107;
t280 = t95+t96+t97;
t281 = t105+t106+t107+t108+t113+t114+t115;
t282 = t95+t96+t97+t98+t119+t120+t121;
t283 = mtd.*slength.*t2.*t53.*(1.0./6.4e1);
t284 = mtd.*slength.*t2.*t55.*(1.0./6.4e1);
t285 = mtd.*slength.*t2.*t57.*(3.0./1.12e2);
t286 = mtd.*slength.*t2.*t59.*(1.5e1./4.48e2);
t287 = mtd.*slength.*t2.*t61.*(1.0./2.8e1);
t288 = mtd.*slength.*t2.*t63.*(1.5e1./4.48e2);
t289 = mtd.*slength.*t2.*t65.*(3.0./1.12e2);
t290 = t283+t284+t285+t286+t287+t288+t289;
t291 = mtd.*slength.*t2.*t29.*(1.0./6.4e1);
t292 = mtd.*slength.*t2.*t18.*(1.0./6.4e1);
t293 = mtd.*slength.*t2.*t22.*(3.0./1.12e2);
t294 = mtd.*slength.*t2.*t26.*(1.5e1./4.48e2);
t295 = mtd.*slength.*t2.*t33.*(1.0./2.8e1);
t296 = mtd.*slength.*t2.*t37.*(1.5e1./4.48e2);
t297 = mtd.*slength.*t2.*t41.*(3.0./1.12e2);
t298 = t291+t292+t293+t294+t295+t296+t297;
t323 = ll.*mtd.*t2.*t4;
t300 = t95+t96+t97+t98+t119+t120+t121-t277+t278-t323;
t303 = t181.*t271.*2.0;
t304 = t180.*t270.*2.0;
t305 = t303+t304;
t306 = m22.*t305.*(1.0./2.0);
t307 = t175.*t269.*2.0;
t308 = t307-t177.*t268.*2.0;
t309 = m21.*t308.*(1.0./2.0);
t310 = t126-t189;
t311 = t71-c22.*t52.*2.0;
t312 = m22.*t311.*(1.0./2.0);
t313 = t71-t85+t94;
t314 = m21.*t313.*(1.0./2.0);
t315 = t185.*t268.*2.0;
t316 = t315-t186.*t269.*2.0;
t317 = m21.*t316.*(1.0./2.0);
t318 = t185.*t270.*2.0;
t319 = t186.*t271.*2.0;
t320 = t318+t319;
t321 = t317-m22.*t320.*(1.0./2.0);
t322 = t269.*t276.*2.0;
t324 = t322-t268.*t300.*2.0;
t325 = m21.*t324.*(1.0./2.0);
t326 = t271.*t272.*2.0;
t327 = t270.*t274.*2.0;
t328 = m22.*(t326+t327).*(1.0./2.0);
t329 = t325+t328;
t330 = t165.*t280.*2.0;
t331 = t166.*t282.*2.0;
t332 = t168.*t298.*2.0;
t333 = t75+t76+t77+t78+t79+t80+t81+t82-c22.*mtd.*t2.*t52.*2.0;
t334 = m22.*t333.*(1.0./2.0);
t335 = t75+t76+t77;
t336 = mms.*t335.*(1.0./2.0);
t337 = t75+t76+t77+t78+t79+t80+t81;
t338 = mmh.*t337.*(1.0./2.0);
t339 = mtd.*slength.*t2.*t53.*(1.0./3.2e1);
t340 = mtd.*slength.*t2.*t55.*(1.0./3.2e1);
t341 = mtd.*slength.*t2.*t57.*(3.0./5.6e1);
t342 = mtd.*slength.*t2.*t59.*(1.5e1./2.24e2);
t343 = mtd.*slength.*t2.*t63.*(1.5e1./2.24e2);
t344 = mtd.*slength.*t2.*t65.*(3.0./5.6e1);
t345 = t108+t339+t340+t341+t342+t343+t344;
t346 = ms.*t345.*(1.0./2.0);
t347 = c21.*mtd.*t2.*t70.*2.0;
t348 = t75+t76+t77+t78+t79+t80+t81+t82+t347-ll.*mtd.*t2.*t70.*2.0;
t349 = m21.*t348.*(1.0./2.0);
t350 = t334+t336+t338+t346+t349;
t351 = t186.*t272.*2.0;
t352 = t185.*t274.*2.0;
t353 = t351+t352;
t354 = m22.*t353.*(1.0./2.0);
t355 = t186.*t276.*2.0;
t356 = t185.*t300.*2.0;
t357 = t355+t356;
t358 = m21.*t357.*(1.0./2.0);
t359 = t186.*t279.*2.0;
t360 = t185.*t280.*2.0;
t361 = t359+t360;
t362 = mms.*t361.*(1.0./2.0);
t363 = t186.*t281.*2.0;
t364 = t185.*t282.*2.0;
t365 = t363+t364;
t366 = mmh.*t365.*(1.0./2.0);
t367 = t186.*t290.*2.0;
t368 = t185.*t298.*2.0;
t369 = t367+t368;
t370 = ms.*t369.*(1.0./2.0);
t371 = t354+t358+t362+t366+t370;
t372 = t45+t46+t47+t48+t49+t50+t51;
t373 = mmh.*t372.*(1.0./2.0);
t374 = mtd.*slength.*t2.*t29.*(1.0./3.2e1);
t375 = mtd.*slength.*t2.*t18.*(1.0./3.2e1);
t376 = mtd.*slength.*t2.*t22.*(3.0./5.6e1);
t377 = mtd.*slength.*t2.*t26.*(1.5e1./2.24e2);
t378 = mtd.*slength.*t2.*t37.*(1.5e1./2.24e2);
t379 = mtd.*slength.*t2.*t41.*(3.0./5.6e1);
t380 = t98+t374+t375+t376+t377+t378+t379;
t381 = ms.*t380.*(1.0./2.0);
t382 = c22.*mtd.*t2.*t7.*2.0;
t387 = ll.*mtd.*t2.*t7.*2.0;
t383 = t45+t46+t47+t48+t49+t50+t51+t382-t387;
t384 = m22.*t383.*(1.0./2.0);
t385 = t46+t47+t48;
t386 = mms.*t385.*(1.0./2.0);
t388 = c21.*mtd.*t2.*t4.*2.0;
bi = [-dphi.*(t373+t381+t384+t386+m21.*(t45+t46+t47+t48+t49+t50+t51+t388-ll.*mtd.*t2.*t4.*2.0-ll.*mtd.*t2.*t7.*2.0).*(1.0./2.0))-dth.*t218+dx.*t84+da2.*(m21.*(t8+t43-ll.*t4.*2.0).*(1.0./2.0)+m22.*(t8-c22.*t7.*2.0).*(1.0./2.0))+da1.*(t44+t263+t264+t265+t266+t267+m12.*(t11-c12.*t13.*2.0).*(1.0./2.0));da2.*(t312+t314)-da1.*t226+dphi.*t350+dy.*t84+dth.*(t74+mmh.*(t54+t56+t58+t60+t62+t64+t66+t68-ll.*t69.*2.0).*(1.0./2.0)+ms.*(t68+t86+t87+t88+t89+t90+t91+t92-ll.*t69.*2.0).*(1.0./2.0)+m21.*(t54+t56+t58+t60+t62+t64+t66+t68+t71+t85-ll.*t69.*2.0-ll.*t70.*2.0).*(1.0./2.0)+mms.*(t56+t58+t60+t68-ll.*t69.*2.0).*(1.0./2.0)+m22.*(t54+t56+t58+t60+t62+t64+t66+t68+t71-c22.*t52.*2.0-ll.*t69.*2.0).*(1.0./2.0)+m12.*(t68-c12.*t69.*2.0).*(1.0./2.0)+mmh.*(t68-ll.*t69.*2.0).*(1.0./2.0));dth.*(t195+ms.*(t134.^2+t136.^2+t137.^2+t145.^2.*2.0+t146.^2+t148.^2+t149.^2+t150.^2+t151.^2+t152.^2+t153.^2.*2.0+t154.^2+t155.^2+t156.^2+t157.^2+t159.^2+t160.^2+t161.^2+t162.^2+t163.^2)+m12.*(t170.^2.*2.0+t171.^2.*2.0).*(1.0./2.0)+m21.*(t175.^2.*2.0+t177.^2.*2.0).*(1.0./2.0)+m22.*(t180.^2.*2.0+t181.^2.*2.0).*(1.0./2.0)+mmh.*(t166.^2.*2.0+t167.^2.*2.0).*(1.0./2.0)+mmh.*(t172.^2.*2.0+t173.^2.*2.0).*(1.0./2.0)+mms.*(t164.^2.*2.0+t165.^2.*2.0).*(1.0./2.0)+ms.*(t168.^2.*2.0+t169.^2.*2.0).*(1.0./2.0))+da2.*(t306+t309)+dy.*(t74+m22.*(t54+t56+t58+t60+t62+t64+t66+t68+t71-t72-c22.*t52.*2.0).*(1.0./2.0)+m21.*(t54+t56+t58+t60+t62+t64+t66+t68+t71-t72+t85-t94).*(1.0./2.0)+m12.*(t68-t93).*(1.0./2.0)+mmh.*(t68-t72).*(1.0./2.0)+mmh.*(t54+t56+t58+t60+t62+t64+t66+t68-t72).*(1.0./2.0)+ms.*(t68-t72+t86+t87+t88+t89+t90+t91+t92).*(1.0./2.0)+mms.*(t56+t58+t60+t68-t72).*(1.0./2.0))-da1.*t262-dx.*t218+dphi.*(m21.*(t177.*(t95+t96+t97+t98+t119+t120+t121+t278-ll.*mtd.*t2.*t4-ll.*mtd.*t2.*t7).*2.0+t276.*(t90+t109+t110+t111+t112+t116+t117+t118+t126+t174-ll.*t69-ll.*t70).*2.0).*(1.0./2.0)+mmh.*(t331+t281.*(t90+t109+t110+t111+t112+t116+t117+t118-ll.*t69).*2.0).*(1.0./2.0)+ms.*(t332+t290.*(t112+t138+t139+t140+t141+t142+t143+t144-ll.*t69).*2.0).*(1.0./2.0)+mms.*(t330+t279.*(t109+t110+t111+t112-ll.*t69).*2.0).*(1.0./2.0)+m22.*(t272.*(t90+t109+t110+t111+t112+t116+t117+t118+t126-c22.*t52-ll.*t69).*2.0+t274.*(t99+t100+t101+t102+t103+t104+t122+t123+t124+t178-ll.*t7).*2.0).*(1.0./2.0));da2.*t321-dphi.*t371-dth.*t262-dy.*t226+dx.*(t44+t263+t264+t265+t266+t267+m12.*(t11-t201).*(1.0./2.0))+da1.*(I11+I12+t195+m21.*t233.*(1.0./2.0)+m22.*t233.*(1.0./2.0)+mmh.*t233+mms.*t233.*(1.0./2.0)+ms.*t233.*(1.0./2.0)+m12.*(t227.^2.*2.0+t228.^2.*2.0).*(1.0./2.0));dth.*(t306+t309)+dy.*(t312+t314)+da1.*t321+dphi.*t329+dx.*(m22.*(t8-t198).*(1.0./2.0)+m21.*(t8+t43-t302).*(1.0./2.0))+da2.*(I21+I22+m21.*(t268.^2.*2.0+t269.^2.*2.0).*(1.0./2.0)+m22.*(t270.^2.*2.0+t310.^2.*2.0).*(1.0./2.0));da2.*t329-da1.*t371+dy.*t350+dth.*(m22.*(t181.*t272.*2.0+t180.*t274.*2.0).*(1.0./2.0)+m21.*(t175.*t276.*2.0+t177.*t300.*2.0).*(1.0./2.0)+mmh.*(t331+t167.*t281.*2.0).*(1.0./2.0)+mms.*(t330+t164.*t279.*2.0).*(1.0./2.0)+ms.*(t332+t169.*t290.*2.0).*(1.0./2.0))-dx.*(t373+t381+t384+t386+m21.*(t45+t46+t47+t48+t49+t50+t51-t387+t388-ll.*mtd.*t2.*t4.*2.0).*(1.0./2.0))+dphi.*(m22.*(t272.^2.*2.0+t274.^2.*2.0).*(1.0./2.0)+m21.*(t276.^2.*2.0+t300.^2.*2.0).*(1.0./2.0)+mmh.*(t281.^2.*2.0+t282.^2.*2.0).*(1.0./2.0)+mms.*(t279.^2.*2.0+t280.^2.*2.0).*(1.0./2.0)+ms.*(t290.^2.*2.0+t298.^2.*2.0).*(1.0./2.0));0.0;0.0;0.0;0.0];