async function parseJson(w, h, padding) {
    const data = await d3.json('../data/formatted_results.json');
    eoss_results = data[0];
    pretrained_results = data[1];
    model_results = [];
    for (let i = 2; i < data.length; i++) {
        model_results.push(data[i]);
    }

    let svg = d3.select("#plt_window").append('svg')
        .attrs({
            id: 'plt_svg',
            height: h,
            width: w
        });

    let min_cost = 0;
    let min_science = 0;
    let max_cost = 20000;
    let max_science = 1.0;

    let x_scale = d3.scaleLinear().domain([min_science, max_science]).range([padding, w - padding]);
    let y_scale = d3.scaleLinear().domain([min_cost, max_cost]).range([h - padding, padding]);
    let x_axis = d3.axisBottom().scale(x_scale);
    let y_axis = d3.axisLeft().scale(y_scale);

    x_map = function (d) {
        return x_scale(d[0]);
    }
    y_map = function (d) {
        return y_scale(d[1]);
    }

    svg.append("g")
        .attrs({
            class: 'x_axis',
            transform: 'translate(0, ' + (h - padding) + ')'
        })
        .call(x_axis)
        .append("text")
        .attrs({
            class: 'label',
            x: w - padding,
            y: -6,
        })
        .style("text-anchor", "end")
        .text("Science Benefit");

    svg.append("g")
        .attrs({
            transform: 'translate(' + padding + ', 0)'
        })
        .call(y_axis)
        .append("text")
        .attrs({
            class: 'label',
            y: padding / 2,
        })
        .style("text-anchor", "end")
        .text("Cost");


    for (let pt of eoss_results.data) {
        svg.append('circle')
            .attrs({
                class: 'eoss_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 2,
                fill: '#009999'
            })
    }

    for (let pt of pretrained_results.data) {
        svg.append('circle')
            .attrs({
                class: 'pretrained_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 2,
                fill: '#ffaa00'
            })
    }

    let currModel = 0;
    for (let pt of model_results[currModel].data) {
        svg.append('circle')
            .attrs({
                class: 'model_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 2,
                fill: '#dc0055'
            })
    }


    d3.select("#model_num")
        .style('width', w - 3.75 * padding + "px")
        .style('margin-left', padding + "px");

    return [eoss_results, pretrained_results, model_results]
}


function updateModel(model_num) {
    svg = d3.select("#plt_svg");
    d3.selectAll('.model_pt').remove();
    for (let pt of model_results[model_num].data) {
        svg.append('circle')
            .attrs({
                class: 'model_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 2,
                fill: '#dc0055'
            })
    }
    d3.select("#training_step_label").text("Training Step: " + model_num);
}