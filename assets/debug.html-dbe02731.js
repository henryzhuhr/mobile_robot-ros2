import{_ as t,r,o as i,c as l,a as e,b as n,d as o,f as s,e as d}from"./app-691e7cc7.js";const c={},h=d('<h1 id="移动机器人控制-ros2" tabindex="-1"><a class="header-anchor" href="#移动机器人控制-ros2" aria-hidden="true">#</a> 移动机器人控制 ROS2</h1><p>基于 Jetson Nano</p><h2 id="🍽️-准备工作" tabindex="-1"><a class="header-anchor" href="#🍽️-准备工作" aria-hidden="true">#</a> 🍽️ 准备工作</h2><h3 id="摄像头畸变矫正" tabindex="-1"><a class="header-anchor" href="#摄像头畸变矫正" aria-hidden="true">#</a> 摄像头畸变矫正</h3>',4),u={href:"https://docs.opencv.org/2.4/_downloads/pattern.png",target:"_blank",rel:"noopener noreferrer"},p=d(`<p><code>camera-distortion-correction.py</code></p><h2 id="📦-功能包" tabindex="-1"><a class="header-anchor" href="#📦-功能包" aria-hidden="true">#</a> 📦 功能包</h2><h3 id="视频流-cpp-video-streamer" tabindex="-1"><a class="header-anchor" href="#视频流-cpp-video-streamer" aria-hidden="true">#</a> 视频流 cpp_video_streamer</h3><p>目录: <code>src/cpp_video_streamer</code></p><p>包含两个节点</p><ul><li>读取视频节点 <code>node_video_reader</code></li><li>显示视频节点 <code>node_video_viewer</code></li></ul><h4 id="读取视频节点-node-video-reader" tabindex="-1"><a class="header-anchor" href="#读取视频节点-node-video-reader" aria-hidden="true">#</a> 读取视频节点 node_video_reader</h4><p>参数</p><ul><li><code>source</code>: 视频源，可选择如下 <ul><li><code>camera</code>: 默认，从摄像头读取</li><li><code>&lt;file/url&gt;</code>: 文件名或者网络url</li></ul></li></ul><h4 id="显示视频节点-node-video-viewer" tabindex="-1"><a class="header-anchor" href="#显示视频节点-node-video-viewer" aria-hidden="true">#</a> 显示视频节点 node_video_viewer</h4><h2 id="🚧-常见问题和解决方案" tabindex="-1"><a class="header-anchor" href="#🚧-常见问题和解决方案" aria-hidden="true">#</a> 🚧 常见问题和解决方案</h2><h3 id="jetson-nano-上串口权限问题" tabindex="-1"><a class="header-anchor" href="#jetson-nano-上串口权限问题" aria-hidden="true">#</a> Jetson Nano 上串口权限问题</h3><div class="language-bash line-numbers-mode" data-ext="sh"><pre class="language-bash"><code>could not <span class="token function">open</span> port /dev/ttyUSB0: <span class="token punctuation">[</span>Errno <span class="token number">13</span><span class="token punctuation">]</span> Permission denied: &#39;/dev/ttyUSB0
</code></pre><div class="line-numbers" aria-hidden="true"><div class="line-number"></div></div></div><p>把自己的用户加入到dialout组</p><div class="language-bash line-numbers-mode" data-ext="sh"><pre class="language-bash"><code><span class="token function">sudo</span> <span class="token function">usermod</span> <span class="token parameter variable">-aG</span> dialout <span class="token variable">\${<span class="token environment constant">USER</span>}</span>  <span class="token comment"># user 替换为自己的用户名</span>
<span class="token function">sudo</span> <span class="token function">usermod</span> <span class="token parameter variable">-aG</span> dialout zx  <span class="token comment"># user 替换为自己的用户名</span>
<span class="token function">reboot</span>							              <span class="token comment"># 必须要重启一下才会生效</span>
</code></pre><div class="line-numbers" aria-hidden="true"><div class="line-number"></div><div class="line-number"></div><div class="line-number"></div></div></div><h3 id="串口名称问题" tabindex="-1"><a class="header-anchor" href="#串口名称问题" aria-hidden="true">#</a> 串口名称问题</h3><p>不同系统下串口名称可能不一样，需要根据实际情况修改，常见的串口名称如下：</p>`,17),_=d("<li><strong>Linux</strong>: <code>/dev/ttyUSB*</code>, <code>/dev/ttyTHS*</code>(Jetson Nano 板载串口), <code>/dev/ttyACM*</code>(STM32下载/串口线)</li><li><strong>Mac</strong>: <code>/dev/tty.usbserial-*</code>, <code>/dev/tty.usbmodem-*</code>(STM32下载/串口线)</li>",2),v=e("strong",null,"Jetson Nano",-1),b={href:"https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide",target:"_blank",rel:"noopener noreferrer"},m=e("code",null,"/dev/ttyTHS1",-1),f=e("code",null,"UART_TX1",-1),g=e("code",null,"UART_RX1",-1),k={href:"https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide#id-.JetsonNano2GBDeveloperKitUserGuidevbatuu_v1.0-40-PinHeader(J6)",target:"_blank",rel:"noopener noreferrer"},x=e("code",null,"/dev/ttyTHS2",-1),T=e("code",null,"UART_RX2",-1),U=e("code",null,"UART_TX2",-1),N={href:"https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide#id-.JetsonNano2GBDeveloperKitUserGuidevbatuu_v1.0-12-PinButtonHeader(J12)",target:"_blank",rel:"noopener noreferrer"},S=e("blockquote",null,[e("p",null,"这里 UART 编号和网上的教程略有不同，这里参考的是官方文档")],-1),y=e("h2",{id:"🔧-调试记录",tabindex:"-1"},[e("a",{class:"header-anchor",href:"#🔧-调试记录","aria-hidden":"true"},"#"),n(" 🔧 调试记录")],-1),B=e("h3",{id:"车道线自动行进调试",tabindex:"-1"},[e("a",{class:"header-anchor",href:"#车道线自动行进调试","aria-hidden":"true"},"#"),n(" 车道线自动行进调试")],-1),J=e("ul",null,[e("li",null,[n("2021.08.06: "),e("ol",null,[e("li",null,"当前直走没有大问题"),e("li",null,"但是修正航向角时，转向存在转过头的问题，然后只能检测单线，导致无法正确前进"),e("li",null,"需要增加根据单车道线行进的逻辑。是否需要透视变换，把车道线拉垂直？")])])],-1);function R(A,G){const a=r("ExternalLinkIcon");return i(),l("div",null,[h,e("p",null,[n("使用 "),e("a",u,[n("Opencv 官方棋盘格"),o(a)]),n(" 进行相机标定")]),s(" 得出标定板的内部行列交点个数 `6 * 9` "),s(" ![棋盘格标定点](./images/camera-distortion-correction--checkerboard.png) "),p,e("ul",null,[_,e("li",null,[v,n(" (参考 "),e("a",b,[n("User Guide"),o(a)]),n(")，有两组 UART: "),e("ul",null,[e("li",null,[m,n(": "),f,n(" (8) / "),g,n(" (10) ("),e("a",k,[n("40-Pin Header (J6)"),o(a)]),n(")")]),e("li",null,[x,n(": "),T,n(" (3) / "),U,n(" (4) ("),e("a",N,[n("12-Pin Button Header (J12)"),o(a)]),n(")")])]),S])]),y,B,J])}const j=t(c,[["render",R],["__file","debug.html.vue"]]);export{j as default};