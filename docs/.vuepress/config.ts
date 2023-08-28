import { defineUserConfig } from 'vuepress'
import { defaultTheme } from '@vuepress/theme-default'
import { MarkdownOptions } from '@vuepress/markdown'

const markdownOptions: MarkdownOptions = {
  headers: {
    level: [2, 3, 4, 5],
  },
}

export default defineUserConfig({
  lang: 'zh-CN',
  title: 'ROS2移动机器人',
  description: '开发文档',
  base: "/mobile_robot-ros2/",

  markdown: markdownOptions,
  theme: defaultTheme({
    // 在这里进行配置
    repo: 'https://github.com/HenryZhuHR/mobile_robot-ros2',
    sidebarDepth: 3, // 设置根据页面标题自动生成的侧边栏的最大深度

  }),
})