---
name: "mygo-presentation"
description: "视频分镜、HTML演示文稿、PPT风格HTML、幻灯片制作、网页幻灯片、演示素材、HTML幻灯片、html做视频分镜、生成HTML演示文稿"
---

# HTML 演示文稿生成器

生成苹果风/Notion 风的 HTML 演示文稿，用于视频分镜或内容展示。

## 核心样式规范（必须严格遵循）

### 配色方案
| 用途 | 颜色值 | 说明 |
|------|--------|------|
| 主背景色 | `#000000` | 纯黑色 |
| 主文字色 | `#ffffff` | 纯白色 |
| 标题文字色 | `#b98eff` | 紫色 |
| 强调文字色 | `#ffc402` | 黄色 |
| 次要文字色 | `#888888` | 浅灰色 |

### 字体规范
```
系统字体栈（零外部依赖）:
font-family: 'Inter', 'Noto Sans SC', -apple-system, 'PingFang SC', 'Microsoft YaHei', sans-serif
标题权重: 700-800
正文权重: 400-500
```

### 布局规范
- 页面比例: 16:9 (padding: 5vh 7vw)
- 使用 CSS Grid/Flexbox 弹性布局
- 字号使用 clamp() 响应式缩放

## HTML 模板

从零生成 HTML 结构，保存为 `templates/presentation.html`（或用户指定路径）。以下为必须包含的组件：

### 必需组件
1. **Slide Engine** - 分页切换逻辑
2. **Controls** - 底部翻页按钮 + 页码计数器
3. **Progress Bar** - 顶部进度条
4. **Dots** - 右侧导航点
5. **Animations** - 淡入动画

### 可复用组件
| 组件 | 类名 | 用途 |
|------|------|------|
| 卡片 | `.card`, `.card-grid` | 内容块 |
| 徽章 | `.badge` | 标签/分类 |
| 高亮条 | `.highlight-bar` | 强调内容 |
| 对比栏 | `.vs-col .ppt-col / .html-col` | 对比展示 |
| 步骤流 | `.steps .step` | 流程展示 |
| 引用框 | `.quote-block` | 引言/金句 |
| 圆圈图 | `.ipo-circle` | IPO/I-P-O 流程 |
| 列表 | `.bullet-list li::before` | 要点列表 |
| 提示框 | `.prompt-box` | 提示词展示 |
| 发光效果 | `.glow .glow-purple/.glow-yellow` | 装饰背景 |
| 封面元数据 | `.cover-tag` | 标签展示 |
| 图标 | `.icon` | 64x64 白色图标 |

## 图标参考

所有图标使用 Icons8 Glyph Neue 白色样式。URL 模板：

```
https://img.icons8.com/glyph-neue/64/FFFFFF/{icon_name}.png
```

将 `{icon_name}` 替换为所需图标名称（如 `notion`、`home`、`github`、`settings`、`star` 等）。

常用图标示例：

| 图标名称 | URL |
|---------|-----|
| notion | https://img.icons8.com/glyph-neue/64/FFFFFF/notion.png |

## 工作流程

1. **理解需求**: 确定演示主题、核心观点、目标受众
2. **内容规划**: 将内容拆分为 8-15 页，每页聚焦单一主题
3. **结构设计**:
   - 封面页（标题 + 副标题 + 标签）
   - 问题/痛点页
   - 核心内容页（2-8 页）
   - 案例/演示页
   - 总结/结尾页
4. **填充内容**: 根据模板组件填充具体内容
5. **预览调整**: 生成后检查效果并微调

## 分页规划建议

| 位置 | 内容 | 页数 |
|------|------|------|
| 前3页 | 封面 + 痛点 + 发现 | 3页 |
| 中间 | 核心观点 + 案例 | 6-10页 |
| 最后 | 总结 + 结尾 | 2页 |

## 生成命令

```html
<!-- 输出到当前工作区 -->
<!-- 使用 write_to_file 工具保存为 .html 文件 -->
```

## 注意事项

1. 每页内容控制在 3-5 个要点以内，避免信息过载
2. 标题使用紫色 (#b98eff)，关键词使用黄色 (#ffc402) 强调
3. 添加 `.glow` 装饰元素增加视觉层次
4. 动画延迟使用 `.anim:nth-child(n)` 递增
5. 封面页使用 `.badge` 和 `.cover-tag` 增加专业感

## ⚠️ 已知问题 & 解决方案（必须遵循）

### 问题1：字体无法加载（404）
- **原因**：`<link>` 标签加载 Google Fonts，在 `file://` 协议下被浏览器安全策略拦截
- **解决**：使用纯系统字体 `font-family: 'Inter', 'Noto Sans SC', -apple-system, 'PingFang SC', 'Microsoft YaHei', sans-serif`，零外部依赖

### 问题2：翻页后内容空白
- **原因**：JS 用 `element.style.xxx = '...'` 设置 inline style 切换页面，但这些 inline style 无法被 CSS 正确覆盖，导致新页面保持 `opacity: 0`
- **解决**：
  1. CSS 的 `.slide.active` 三个关键属性必须加 `!important`：`opacity`、`visibility`、`transform`
  2. JS 翻页函数**禁止使用 inline style**，只操作 `classList.add/remove('active')` 和 `classList.add('exit-up')`，完全由 CSS transition 处理动画

**必须使用的 CSS 代码**：
```css
.slide {
  position: absolute;
  inset: 0;
  opacity: 0;
  visibility: hidden;
  transform: translateY(30px);
  transition: opacity 0.5s ease, transform 0.5s ease, visibility 0.5s ease;
}
.slide.active {
  opacity: 1 !important;
  visibility: visible !important;
  transform: translateY(0) !important;
}
.slide.exit-up {
  opacity: 0 !important;
  transform: translateY(-30px) !important;
}
```

**必须使用的 JS 翻页函数**：
```javascript
function goToSlide(index) {
  if (index === currentSlide || index < 0 || index >= slides.length) return;
  // 旧页退场
  slides[currentSlide].classList.remove('active');
  slides[currentSlide].classList.add('exit-up');
  // 新页登场
  slides[index].classList.remove('exit-up');
  slides[index].classList.add('active');
  currentSlide = index;
  updateControls();
}
```

