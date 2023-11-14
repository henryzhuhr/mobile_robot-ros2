import { defineUserConfig } from 'vuepress'
import { DefaultThemeOptions, defaultTheme } from '@vuepress/theme-default'
import { MarkdownOptions } from '@vuepress/markdown'

// set markdown options
const markdownOptions: MarkdownOptions = {
  headers: {
    level: [2, 3, 4, 5],
  },
}

// set theme config
const themeOptions: DefaultThemeOptions = {
  // 在这里进行配置
  repo: 'https://github.com/HenryZhuHR/mobile_robot-ros2',
  sidebarDepth: 3, // 设置根据页面标题自动生成的侧边栏的最大深度
}

export default defineUserConfig({
  title: 'ROS2移动机器人',
  description: '开发文档',

  // set site base to default value
  base: "/mobile_robot-ros2/",

  // site-level locales config
  lang: 'zh-CN',

  // configure default theme
  theme: defaultTheme(themeOptions),

  // configure markdown
  markdown: markdownOptions,
})