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
    title: 'ROS2 开发',
    description: 'ROS2 开发文档',
    base: "/ros2-development/",

    markdown: markdownOptions,
  theme: defaultTheme({
    // 在这里进行配置
    repo: 'https://github.com/HenryZhuHR/interview-preparation',
    sidebarDepth: 2, // 设置根据页面标题自动生成的侧边栏的最大深度

  }),
})