function ddf_torqueddth = ddf_torqueddthFunc6(in1,in2,in3)
%DDF_TORQUEDDTHFUNC6
%    DDF_TORQUEDDTH = DDF_TORQUEDDTHFUNC6(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:11:33

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
L5 = in2(5,:);
L6 = in2(6,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
th6 = in1(6,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = conj(th3);
t5 = conj(th4);
t6 = conj(th5);
t7 = t2+t3+t4+t5+t6;
t8 = sin(t7);
t10 = L5.*4.16925;
t11 = t10+3.5316;
t27 = L5.*(1.7e1./4.0e1);
t28 = t27+9.0./2.5e1;
t29 = 1.0./t28;
t30 = L5.*(9.0./2.5e1);
t31 = L5.^2;
t32 = t31.*(1.7e1./8.0e1);
t33 = t30+t32;
t39 = conj(th6);
t40 = t2+t3+t4+t5+t6+t39;
t73 = sin(t40);
t80 = L6.*t73.*(1.0./2.0);
t81 = L5.*t8;
t157 = t80+t81;
t158 = L6.*t157.*4.16925;
t159 = t8.*t11.*t29.*t33;
t9 = t158+t159;
t12 = t2+t3+t4+t5;
t13 = cos(t12);
t14 = L4.*(1.7e1./4.0e1);
t15 = t14+9.0./2.5e1;
t16 = L4.*(9.0./2.5e1);
t17 = L4.^2;
t18 = t17.*(1.7e1./8.0e1);
t19 = t16+t18;
t21 = cos(t2);
t22 = t2+t3;
t23 = cos(t22);
t25 = t2+t3+t4;
t26 = cos(t25);
t34 = L4.*4.16925;
t35 = t34+3.5316;
t38 = 1.0./t15;
t68 = L1.*t21.*4.16925;
t104 = 1.0./t15.^2;
t114 = L2.*t23.*4.16925;
t133 = L3.*t26.*4.16925;
t255 = t11.*t13;
t256 = L6.*t13.*4.16925;
t257 = t13.*t19.*t104.*(1.7e1./4.0e1);
t258 = t13-t257;
t259 = t35.*t258;
t260 = t13.*t19.*t38.*4.16925;
t20 = t68+t114+t133+t255+t256+t259+t260;
t24 = L1.*t21;
t36 = L2.*t23;
t37 = L3.*t26;
t41 = cos(t7);
t42 = L4.*t13;
t44 = t29.*t33.*t41;
t45 = t13.*t19.*t38;
t46 = cos(t40);
t47 = L6.*t46.*(1.0./2.0);
t48 = L5.*t41;
t49 = L3.*(1.7e1./4.0e1);
t50 = t49+9.0./2.5e1;
t51 = 1.0./t50;
t52 = L3.*4.16925;
t53 = t52+3.5316;
t54 = L3.*(9.0./2.5e1);
t55 = L3.^2;
t56 = t55.*(1.7e1./8.0e1);
t57 = t54+t56;
t60 = L2.*4.16925;
t61 = t60+3.5316;
t62 = L2.*(1.7e1./4.0e1);
t63 = t62+9.0./2.5e1;
t64 = L2.*(9.0./2.5e1);
t65 = L2.^2;
t66 = t65.*(1.7e1./8.0e1);
t67 = t64+t66;
t69 = 1.0./t63;
t84 = L1.*(1.7e1./4.0e1);
t85 = t84+9.0./2.5e1;
t86 = 1.0./t85;
t87 = L1.*4.16925;
t88 = t87+3.5316;
t89 = L1.*(9.0./2.5e1);
t90 = L1.^2;
t91 = t90.*(1.7e1./8.0e1);
t92 = t89+t91;
t131 = t26.*t51.*t57;
t196 = t23.*t67.*t69;
t197 = t24+t196;
t198 = t61.*t197;
t199 = t24+t36+t37+t42+t44;
t200 = t11.*t199;
t201 = t24+t36+t37+t45;
t202 = t35.*t201;
t203 = t24+t36+t37+t42+t47+t48;
t204 = L6.*t203.*4.16925;
t205 = t24+t36+t131;
t206 = t53.*t205;
t207 = t21.*t86.*t88.*t92;
t43 = t198+t200+t202+t204+t206+t207;
t162 = t37+t42+t44;
t163 = t11.*t162;
t164 = t37+t45;
t165 = t35.*t164;
t166 = t37+t42+t47+t48;
t167 = L6.*t166.*4.16925;
t168 = t26.*t51.*t53.*t57;
t58 = t163+t165+t167+t168;
t180 = t42+t44;
t181 = t11.*t180;
t182 = t42+t47+t48;
t183 = L6.*t182.*4.16925;
t184 = t13.*t19.*t35.*t38;
t59 = t181+t183+t184;
t144 = 1.0./t63.^2;
t172 = L6.*t23.*4.16925;
t173 = t23.*t67.*t144.*(1.7e1./4.0e1);
t174 = t23-t173;
t175 = t61.*t174;
t176 = t23.*t53;
t177 = t23.*t35;
t178 = t11.*t23;
t179 = t23.*t67.*t69.*4.16925;
t70 = t68+t172+t175+t176+t177+t178+t179;
t71 = L6.^2;
t72 = t71.^2;
t74 = sin(t2);
t75 = L1.*t74;
t76 = sin(t22);
t77 = L2.*t76;
t78 = sin(t25);
t79 = L3.*t78;
t82 = sin(t12);
t83 = L4.*t82;
t126 = t19.*t38.*t82;
t127 = t51.*t57.*t78;
t128 = t8.*t29.*t33;
t219 = t75+t77+t79+t126;
t220 = t35.*t219;
t221 = t67.*t69.*t76;
t222 = t75+t221;
t223 = t61.*t222;
t224 = t75+t77+t127;
t225 = t53.*t224;
t226 = t75+t77+t79+t80+t81+t83;
t227 = L6.*t226.*4.16925;
t228 = t75+t77+t79+t83+t128;
t229 = t11.*t228;
t230 = t74.*t86.*t88.*t92;
t93 = t220+t223+t225+t227+t229+t230;
t94 = 1.0./t85.^2;
t95 = t74-t74.*t92.*t94.*(1.7e1./4.0e1);
t96 = t88.*t95;
t97 = t61.*t74;
t98 = t53.*t74;
t99 = t35.*t74;
t100 = t11.*t74;
t101 = L6.*t74.*4.16925;
t102 = t74.*t86.*t92.*4.16925;
t103 = L6.*t21.*4.16925+t11.*t21+t21.*t35+t21.*t53+t21.*t61+t88.*(t21-t21.*t92.*t94.*(1.7e1./4.0e1))+t21.*t86.*t92.*4.16925;
t105 = yb.*4.16925;
t106 = L3.*t78.*4.16925;
t231 = t19.*t82.*t104.*(1.7e1./4.0e1);
t107 = t82-t231;
t108 = t35.*t107;
t109 = L2.*t76.*4.16925;
t110 = t11.*t82;
t111 = L6.*t82.*4.16925;
t112 = L1.*t74.*4.16925;
t113 = t19.*t38.*t82.*4.16925;
t119 = 1.0./t50.^2;
t263 = t26.*t57.*t119.*(1.7e1./4.0e1);
t264 = t26-t263;
t265 = t53.*t264;
t266 = t26.*t35;
t267 = t11.*t26;
t268 = L6.*t26.*4.16925;
t269 = t26.*t51.*t57.*4.16925;
t115 = t68+t114+t265+t266+t267+t268+t269;
t116 = L6.*t73.*4.16925;
t117 = L5.*t8.*4.16925;
t118 = L4.*t82.*4.16925;
t120 = L6.*t78.*4.16925;
t236 = t57.*t78.*t119.*(1.7e1./4.0e1);
t121 = t78-t236;
t122 = t53.*t121;
t123 = t35.*t78;
t124 = t11.*t78;
t125 = t51.*t57.*t78.*4.16925;
t210 = t77+t79+t126;
t211 = t35.*t210;
t212 = t77+t127;
t213 = t53.*t212;
t214 = t77+t79+t80+t81+t83;
t215 = L6.*t214.*4.16925;
t216 = t77+t79+t83+t128;
t217 = t11.*t216;
t218 = t61.*t67.*t69.*t76;
t129 = t211+t213+t215+t217+t218;
t238 = t80+t81+t83;
t239 = L6.*t238.*4.16925;
t240 = t83+t128;
t241 = t11.*t240;
t242 = t19.*t35.*t38.*t82;
t130 = t239+t241+t242;
t187 = t36+t37+t42+t47+t48;
t188 = L6.*t187.*4.16925;
t189 = t36+t131;
t190 = t53.*t189;
t191 = t36+t37+t42+t44;
t192 = t11.*t191;
t193 = t36+t37+t45;
t194 = t35.*t193;
t195 = t23.*t61.*t67.*t69;
t132 = t188+t190+t192+t194+t195;
t135 = L4.*t13.*4.16925;
t138 = 1.0./t28.^2;
t152 = t33.*t41.*t138.*(1.7e1./4.0e1);
t153 = t41-t152;
t154 = t11.*t153;
t155 = L6.*t41.*4.16925;
t156 = t29.*t33.*t41.*4.16925;
t134 = t68+t114+t133+t135+t154+t155+t156;
t170 = L6.*t46.*4.16925;
t171 = L5.*t41.*4.16925;
t136 = t68+t114+t133+t135+t170+t171;
t246 = t79+t80+t81+t83;
t247 = L6.*t246.*4.16925;
t248 = t79+t83+t128;
t249 = t11.*t248;
t250 = t79+t126;
t251 = t35.*t250;
t252 = t51.*t53.*t57.*t78;
t137 = t247+t249+t251+t252;
t139 = L6.*t8.*4.16925;
t261 = t8.*t33.*t138.*(1.7e1./4.0e1);
t140 = t8-t261;
t141 = t11.*t140;
t142 = t8.*t29.*t33.*4.16925;
t270 = t47+t48;
t271 = L6.*t270.*4.16925;
t272 = t11.*t29.*t33.*t41;
t143 = t271+t272;
t145 = L6.*t76.*4.16925;
t274 = t67.*t76.*t144.*(1.7e1./4.0e1);
t146 = t76-t274;
t147 = t61.*t146;
t148 = t53.*t76;
t149 = t35.*t76;
t150 = t11.*t76;
t151 = t67.*t69.*t76.*4.16925;
t160 = t9.^2;
t161 = t160.*2.0;
t169 = t58.^2;
t185 = t59.^2;
t186 = t46.^2;
t208 = t73.^2;
t209 = t72.*t208.*8.69132278125;
t232 = t105+t106+t108+t109+t110+t111+t112+t113;
t233 = t129.^2;
t234 = t233.*2.0;
t235 = t105+t106+t109+t112+t116+t117+t118;
t237 = t105+t109+t112+t120+t122+t123+t124+t125;
t243 = t130.^2;
t244 = t243.*2.0;
t245 = t132.^2;
t253 = t137.^2;
t254 = t253.*2.0;
t262 = t105+t106+t109+t112+t118+t139+t141+t142;
t273 = t143.^2;
t275 = t105+t112+t145+t147+t148+t149+t150+t151;
t276 = t114+t133+t135+t154+t155+t156;
t277 = t134.*t276.*2.0;
t278 = t114+t133+t135+t170+t171;
t279 = t136.*t278.*2.0;
t280 = t172+t175+t176+t177+t178+t179;
t281 = t70.*t280.*2.0;
t282 = t93.*t129.*2.0;
t283 = t106+t108+t109+t110+t111+t113;
t284 = t106+t109+t116+t117+t118;
t285 = t109+t120+t122+t123+t124+t125;
t286 = t114+t133+t255+t256+t259+t260;
t287 = t20.*t286.*2.0;
t288 = t106+t109+t118+t139+t141+t142;
t289 = t114+t265+t266+t267+t268+t269;
t290 = t115.*t289.*2.0;
t291 = t145+t147+t148+t149+t150+t151;
t293 = t169.*2.0;
t294 = t185.*2.0;
t295 = t72.*t186.*8.69132278125;
t296 = t43.*t132.*2.0;
t297 = t232.*t283.*2.0;
t298 = t235.*t284.*2.0;
t299 = t237.*t285.*2.0;
t300 = t245.*2.0;
t301 = t262.*t288.*2.0;
t302 = t273.*2.0;
t303 = t275.*t291.*2.0;
t292 = t161+t209+t234+t244+t254+t277+t279+t281+t282+t287+t290-t293-t294-t295-t296-t297-t298-t299-t300-t301-t302-t303;
t304 = t133+t135+t170+t171;
t305 = t106+t108+t110+t111+t113;
t306 = t106+t116+t117+t118;
t307 = t120+t122+t123+t124+t125;
t308 = t106+t118+t139+t141+t142;
t309 = t133+t255+t256+t259+t260;
t310 = t265+t266+t267+t268+t269;
t311 = t133+t135+t154+t155+t156;
t312 = t135+t154+t155+t156;
t313 = t135+t170+t171;
t314 = t108+t110+t111+t113;
t315 = t116+t117+t118;
t316 = t118+t139+t141+t142;
t317 = t255+t256+t259+t260;
t318 = t130.*t137.*2.0;
t319 = t154+t155+t156;
t320 = t9.*t130.*2.0;
t321 = t170+t171;
t322 = t9.*t137.*2.0;
t323 = t116+t117;
t324 = t139+t141+t142;
t325 = t71.*t73.*t130.*4.16925;
t326 = t71.*t73.*t137.*4.16925;
t327 = t9.*t71.*t73.*4.16925;
t328 = t20.*t309.*2.0;
t329 = t93.*t137.*2.0;
t330 = t115.*t310.*2.0;
t331 = t129.*t137.*2.0;
t332 = t134.*t311.*2.0;
t333 = t136.*t304.*2.0;
t334 = t278.*t304.*2.0;
t335 = t286.*t309.*2.0;
t336 = t289.*t310.*2.0;
t337 = t129.*t137.*4.0;
t338 = t276.*t311.*2.0;
t340 = t58.*t132.*2.0;
t341 = t232.*t305.*2.0;
t342 = t235.*t306.*2.0;
t343 = t237.*t307.*2.0;
t344 = t262.*t308.*2.0;
t345 = t43.*t58.*2.0;
t339 = t161+t209+t244+t254-t293-t294-t295-t302+t334+t335+t336+t337+t338-t340-t341-t342-t343-t344-t345;
t346 = t93.*t130.*2.0;
t347 = t20.*t317.*2.0;
t348 = t129.*t130.*2.0;
t349 = t134.*t312.*2.0;
t350 = t136.*t313.*2.0;
t351 = t276.*t312.*2.0;
t352 = t278.*t313.*2.0;
t353 = t286.*t317.*2.0;
t354 = t129.*t130.*4.0;
t356 = t232.*t314.*2.0;
t357 = t235.*t315.*2.0;
t358 = t59.*t132.*2.0;
t359 = t262.*t316.*2.0;
t361 = t43.*t59.*2.0;
t363 = t58.*t59.*2.0;
t355 = t161+t209+t244-t294-t295-t302+t318+t351+t352+t353+t354-t356-t357-t358-t359-t361-t363;
t360 = t309.*t317.*2.0;
t362 = t130.*t137.*6.0;
t364 = t311.*t312.*2.0;
t365 = t304.*t313.*2.0;
t366 = t134.*t319.*2.0;
t367 = t136.*t321.*2.0;
t368 = t9.*t129.*2.0;
t369 = t9.*t93.*2.0;
t370 = t9.*t129.*4.0;
t371 = t276.*t319.*2.0;
t372 = t278.*t321.*2.0;
t374 = t132.*t143.*2.0;
t378 = t43.*t143.*2.0;
t379 = t58.*t143.*2.0;
t380 = t235.*t323.*2.0;
t381 = t59.*t143.*2.0;
t382 = t262.*t324.*2.0;
t373 = t161+t209-t295-t302+t320+t322+t370+t371+t372-t374-t378-t379-t380-t381-t382;
t375 = t311.*t319.*2.0;
t376 = t304.*t321.*2.0;
t377 = t9.*t137.*6.0;
t383 = t312.*t319.*2.0;
t384 = t313.*t321.*2.0;
t385 = t9.*t130.*8.0;
t386 = t71.*t73.*t129.*4.16925;
t387 = L6.*t46.*t136.*8.3385;
t388 = t71.*t73.*t93.*4.16925;
t389 = t71.*t73.*t129.*8.3385;
t390 = L6.*t46.*t278.*8.3385;
t392 = L6.*t73.*t235.*8.3385;
t393 = t46.*t71.*t132.*4.16925;
t395 = t46.*t71.*t143.*4.16925;
t396 = t43.*t46.*t71.*4.16925;
t397 = t46.*t58.*t71.*4.16925;
t398 = t46.*t59.*t71.*4.16925;
t391 = t209-t295+t325+t326+t327+t389+t390-t392-t393-t395-t396-t397-t398;
t394 = t71.*t73.*t137.*1.250775e1;
t399 = L6.*t46.*t304.*8.3385;
t400 = t71.*t73.*t130.*1.6677e1;
t401 = L6.*t46.*t313.*8.3385;
t402 = t9.*t71.*t73.*2.084625e1;
t403 = L6.*t46.*t321.*8.3385;
ddf_torqueddth = reshape([t161-t169.*2.0-t185.*2.0+t209+t234+t244-t245.*2.0+t254-t273.*2.0-t72.*t186.*8.69132278125-t232.*(t106+t108+t109+t110+t111+t112+t113).*2.0-t237.*(t109+t112+t120+t122+t123+t124+t125).*2.0-t262.*(t106+t109+t112+t118+t139+t141+t142).*2.0-t275.*(t112+t145+t147+t148+t149+t150+t151).*2.0+t20.^2.*2.0-t43.^2.*2.0+t70.^2.*2.0+t93.^2.*2.0+t103.^2.*2.0+t115.^2.*2.0+t134.^2.*2.0+t136.^2.*2.0-(t96+t97+t98+t99+t100+t101+t102).*(t96+t97+t98+t99+t100+t101+t102+t105).*2.0-t235.*(t106+t109+t112+t116+t117+t118).*2.0,t292,t161+t209+t244+t254-t293-t294-t295-t302+t328+t329+t330+t331+t332+t333-t43.*t58.*2.0-t58.*t132.*2.0-t232.*t305.*2.0-t235.*t306.*2.0-t237.*t307.*2.0-t262.*t308.*2.0,t161+t209+t244-t294-t295-t302+t318+t346+t347+t348+t349+t350-t43.*t59.*2.0-t58.*t59.*2.0-t59.*t132.*2.0-t232.*t314.*2.0-t235.*t315.*2.0-t262.*t316.*2.0,t161+t209-t295-t302+t320+t322+t366+t367+t368+t369-t43.*t143.*2.0-t58.*t143.*2.0-t59.*t143.*2.0-t132.*t143.*2.0-t235.*t323.*2.0-t262.*t324.*2.0,t209-t295+t325+t326+t327+t386+t387+t388-L6.*t73.*t235.*8.3385-t43.*t46.*t71.*4.16925-t46.*t58.*t71.*4.16925-t46.*t59.*t71.*4.16925-t46.*t71.*t132.*4.16925-t46.*t71.*t143.*4.16925,t292,t161+t209+t233.*4.0+t244+t254-t293-t294-t295-t296-t297-t298-t299-t300-t301-t302-t303+t276.^2.*2.0+t278.^2.*2.0+t280.^2.*2.0+t286.^2.*2.0+t289.^2.*2.0,t339,t355,t373,t391,t161-t169.*2.0-t185.*2.0+t209+t244+t254-t273.*2.0+t328+t329+t330+t331+t332+t333-t43.*t58.*2.0-t58.*t132.*2.0-t72.*t186.*8.69132278125-t232.*t305.*2.0-t235.*t306.*2.0-t237.*t307.*2.0-t262.*t308.*2.0,t339,t161+t209+t244+t253.*6.0-t293-t294-t295-t302-t340-t341-t342-t343-t344-t345+t304.^2.*2.0+t309.^2.*2.0+t310.^2.*2.0+t311.^2.*2.0,t161+t209+t244-t294-t295-t302-t356-t357-t358-t359+t360-t361+t362-t363+t364+t365,t161+t209-t295-t302+t320-t374+t375+t376+t377-t378-t379-t380-t381-t382,t209-t295+t325+t327-t392-t393+t394-t395-t396-t397-t398+t399,t161-t185.*2.0+t209+t244-t273.*2.0+t318+t346+t347+t348+t349+t350-t43.*t59.*2.0-t58.*t59.*2.0-t59.*t132.*2.0-t72.*t186.*8.69132278125-t232.*t314.*2.0-t235.*t315.*2.0-t262.*t316.*2.0,t355,t161+t209+t244-t294-t295-t302+t360+t362+t364+t365-t43.*t59.*2.0-t58.*t59.*2.0-t59.*t132.*2.0-t232.*t314.*2.0-t235.*t315.*2.0-t262.*t316.*2.0,t161+t209+t243.*8.0-t294-t295-t302-t356-t357-t358-t359-t361-t363+t312.^2.*2.0+t313.^2.*2.0+t317.^2.*2.0,t161+t209-t295-t302-t374-t378-t379-t380-t381-t382+t383+t384+t385,t209-t295+t327-t392-t393-t395-t396-t397-t398+t400+t401,t161+t209-t273.*2.0+t320+t322+t366+t367+t368+t369-t43.*t143.*2.0-t58.*t143.*2.0-t59.*t143.*2.0-t72.*t186.*8.69132278125-t132.*t143.*2.0-t235.*t323.*2.0-t262.*t324.*2.0,t373,t161+t209-t295-t302+t320+t375+t376+t377-t43.*t143.*2.0-t58.*t143.*2.0-t59.*t143.*2.0-t132.*t143.*2.0-t235.*t323.*2.0-t262.*t324.*2.0,t161+t209-t295-t302+t383+t384+t385-t43.*t143.*2.0-t58.*t143.*2.0-t59.*t143.*2.0-t132.*t143.*2.0-t235.*t323.*2.0-t262.*t324.*2.0,t160.*1.0e1+t209-t295-t302-t374-t378-t379-t380-t381-t382+t319.^2.*2.0+t321.^2.*2.0,t209-t295-t392-t393-t395-t396-t397-t398+t402+t403,t209+t325+t326+t327+t386+t387+t388-t72.*t186.*8.69132278125-L6.*t73.*t235.*8.3385-t43.*t46.*t71.*4.16925-t46.*t58.*t71.*4.16925-t46.*t59.*t71.*4.16925-t46.*t71.*t132.*4.16925-t46.*t71.*t143.*4.16925,t391,t209-t295+t325+t327+t394+t399-L6.*t73.*t235.*8.3385-t43.*t46.*t71.*4.16925-t46.*t58.*t71.*4.16925-t46.*t59.*t71.*4.16925-t46.*t71.*t132.*4.16925-t46.*t71.*t143.*4.16925,t209-t295+t327+t400+t401-L6.*t73.*t235.*8.3385-t43.*t46.*t71.*4.16925-t46.*t58.*t71.*4.16925-t46.*t59.*t71.*4.16925-t46.*t71.*t132.*4.16925-t46.*t71.*t143.*4.16925,t209-t295+t402+t403-L6.*t73.*t235.*8.3385-t43.*t46.*t71.*4.16925-t46.*t58.*t71.*4.16925-t46.*t59.*t71.*4.16925-t46.*t71.*t132.*4.16925-t46.*t71.*t143.*4.16925,-t295-t392-t393-t395-t396-t397-t398+t71.*t186.*3.4765291125e1+t72.*t208.*5.21479366875e1],[6,6]);
