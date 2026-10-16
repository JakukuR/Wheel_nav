'use strict';
const featureImage = document.getElementById('image');
const featureStatus = document.getElementById('status');
const featureDetail = document.getElementById('detail');
const featureViewer = document.getElementById('viewer');
const featureEmpty = document.getElementById('empty');
let featureSequence = -1;
let featureURL = null;

async function pollFeatures() {
  try {
    if (!document.hidden) {
      const response = await fetch('/api/vio/status', {cache:'no-store', signal:AbortSignal.timeout(2500)});
      if (!response.ok) throw new Error('status unavailable');
      const state = await response.json();
      featureStatus.textContent = state.live ? '实时图像' : (state.sequence ? '图像已停更' : '等待图像');
      featureStatus.classList.toggle('live', Boolean(state.live));
      featureViewer.classList.toggle('stale', !state.live);
      featureEmpty.hidden = Boolean(state.live);
      featureEmpty.textContent = state.sequence ? '画面已停更，当前为最后一帧。' : '等待 cuVSLAM 图像。请先启动 cuVSLAM 导航。';
      if (state.sequence) {
        const date = new Date(state.stamp * 1000).toLocaleTimeString('zh-CN', {hour12:false});
        featureDetail.textContent = `${state.width} × ${state.height} · 采集时间 ${date} · 图像年龄 ${state.source_age_s.toFixed(2)} s`;
        if (state.sequence !== featureSequence) {
          const frame = await fetch('/api/vio/image.jpg', {cache:'no-store', signal:AbortSignal.timeout(2500)});
          if (!frame.ok) throw new Error('image unavailable');
          const url = URL.createObjectURL(await frame.blob());
          const previous = featureURL;
          featureImage.src = url;
          featureImage.hidden = false;
          featureURL = url;
          if (previous) URL.revokeObjectURL(previous);
          featureSequence = state.sequence;
        }
      }
    }
  } catch (_) {
    featureStatus.textContent = '连接中断';
    featureStatus.classList.remove('live');
    featureViewer.classList.add('stale');
    featureEmpty.hidden = false;
    featureEmpty.textContent = '无法连接导航页面服务。';
  } finally {
    window.setTimeout(pollFeatures, 200);
  }
}
pollFeatures();
