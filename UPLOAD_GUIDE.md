# GitHub 上传步骤

建议仓库名：`femoral-head-indexing-station`

1. 解压 ZIP，打开同名文件夹。此层应能看到 `README.md`、`firmware`、`archive`、`diagnostics`、`course-examples` 和 `docs`。
2. 在 GitHub 创建 Public 仓库。简介可复制 `ABOUT.txt`；README、.gitignore 已有，创建时无需自动生成，License 先留为 None。
3. 在空仓库点击 **uploading an existing file**；已有仓库使用 **Add file → Upload files**。
4. 全选解压后项目文件夹里面的内容，拖到上传区域。确保上传路径从 `README.md`、`firmware/...` 开始，没有多包一层目录。
5. 提交信息填写 `Add femoral head indexing station project`，提交到新仓库主分支；若仓库规则要求 PR，按页面提示完成。
6. 确认首页 README 和两张图片正常显示，在 About 中添加 `https://shenghan.cc/projects.html`。

可选 topics：`arduino`、`cpp`、`raspberry-pi-pico`、`pid-control`、`encoder`、`mechatronics`、`motor-control`。

主程序是 `firmware/anti_stall_encoder_sign_flip/anti_stall_encoder_sign_flip.ino`。全部程序保持原样，包括你确认成功版本中的 116°、−99°、45% duty 上限和 encoder sign −1。

上传解压后的内容，GitHub 才会直接展示代码与 README；上传 ZIP 本身只会存放一个压缩包。

官方参考：https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository
