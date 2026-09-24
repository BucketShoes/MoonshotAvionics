# Vendored chart libraries

Local copies of the chart stack so the dashboard works with **no internet
access** — the whole point of the PWA (`../sw.js`). `app.js` loads these first
and only falls back to jsDelivr if a local file is missing (which is the case
when the page is served from the ESP32's LittleFS, since `compress_web.ps1`
does not upload this folder).

| File | Package | Version | Licence |
|---|---|---|---|
| `chart.umd.min.js` | [chart.js](https://www.chartjs.org/) | 4.5.1 | MIT |
| `hammer.min.js` | [hammerjs](https://hammerjs.github.io/) | 2.0.8 | MIT |
| `chartjs-plugin-zoom.min.js` | [chartjs-plugin-zoom](https://www.chartjs.org/chartjs-plugin-zoom/) | 2.2.0 | MIT |

To refresh:

```sh
npm pack chart.js@4 hammerjs@2 chartjs-plugin-zoom@2
# extract, then copy:
#   chart.js/dist/chart.umd.min.js
#   hammerjs/hammer.min.js
#   chartjs-plugin-zoom/dist/chartjs-plugin-zoom.min.js
```

Then bump `CACHE_VERSION` in `../sw.js` and update the versions above.

> The folder is named `lib/`, not `vendor/`, on purpose: Jekyll's default
> exclude list drops `vendor/`, so a `vendor/` folder would silently 404 on
> GitHub Pages. `docs/.nojekyll` also disables Jekyll for the same reason.
