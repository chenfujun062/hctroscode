# 同屏器升级服务

## 1 部署说明

### 1.1 服务启动

* 直接解压缩，并运行`elf_upgrade.exe`即可运行服务
  * Linux版本则是`elf_upgrade`
* 请服务器预留`8000`端口，并打开防火墙设置等
  * 端口号可以通过配置进行配置

### 1.2 文件说明

* `elf_upgrade.exe`：服务可执行文件
* `configs`: 配置目录
  * `main.json`：主配置文件
* `packages`：升级包目录
  * `link`：有线升级包目录
  * `network`：网络升级包目录
* `runtime`：运行时产生文件的防止目录

### 1.3 配置文件项说明

* 配置文件：`config/main.json`
* 配置项说明：
  * `_servicePort`：服务占用端口
  * `_packagesDir`：升级包放置目录（默认为根目下得`packages`）
  * `enableLatestVersionUpgrade`：在客户端已为最新版本的情况下，是否运行继续升级

### 1.4 升级包放置规范

* 根据有线升级/网络升级，把升级包放置到对应目录中：
  * 有线：`packages/link`
  * 无限：`packages/network`
* 并且根据规范进行命名：
  * 格式: `UGP_[产品型号]_[版本号].bin`
  * 例如：`UPG_A210_1.0.0-150420.bin`

## 2 使用说明

### 2.1 有线升级

1. 手机使用USB线连接设备
2. 手机使用浏览器打开刚才部署的服务：http://<ip>:<port>/link/upgrade.html?product={产品号}&version={当前客户端版本}&chip={芯片ID}&vendor={厂商}
   * ip为部署的服务器IP或域名，port为上面所配置的端口
   * 参数中product为必需参数，其他参数可根据实际情况省略
   * 在实际生产使用中，可把服务地址转化为二维码显示，直接供用户扫描访问
3. 满足条件并访问成功后，直接点击网页中的开始升级即可

### 2.2 网络升级

1. 手机需要和设备处于同一网段
   * 服务需要通过局域网读取设备信息
2. 手机使用浏览器打开刚才部署的服务：http://<ip>:<port>/network/upgrade.html
   * 在实际生产使用中，可把服务地址转化为二维码显示，直接供用户扫描访问
3. 满足条件并访问成功后，直接点击网页中的开始升级即可