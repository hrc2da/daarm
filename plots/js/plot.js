async function parseJson(filename, w, h, padding) {
    const data = await d3.json(filename);
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
    let max_cost = 10000;
    let max_science = 1;

    let x_scale = d3.scaleLinear().domain([min_science, max_science]).range([padding, w - padding]);
    let y_scale = d3.scaleLinear().domain([min_cost, max_cost]).range([h - padding, padding]);
    let x_axis = d3.axisBottom().scale(x_scale);
    let y_axis = d3.axisLeft().scale(y_scale);

    //map functions for scaling pts
    x_map = function (d) {
        let input = Math.max(Number(d[0]), 0);
        return x_scale(input);
    }
    y_map = function (d) {
        let input = Math.max(Number(d[1]), 0);
        return y_scale(input);
    }

    //place x-axis on svg
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

    //place y-axis on svg
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


    //plot eoss plot
    for (let pt of eoss_results.data) {
        svg.append('circle')
            .attrs({
                class: 'eoss_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 4,
                fill: '#009999',
                opacity: 0.7
            })
        svg.append('circle')
            .attrs({
                class: 'eoss_pt',
                cx: x_map(pt),
                cy: y_map(pt),
                r: 2,
                fill: '#ffffff'
            })
    }
    //plot pretrained plot
    for (let pt of pretrained_results.data) {
        svg.append('rect')
            .attrs({
                class: 'pretrained_pt',
                x: x_map(pt),
                y: y_map(pt),
                width: 5,
                height: 5,
                fill: '#ffaa00',
                opacity: 0.7
            })
    }
    //plot first model plot
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

    //define position of slider based on size of plot
    d3.select("#model_num")
        .style('width', w - 3.75 * padding + "px")
        .style('margin-left', padding + "px");

    return [eoss_results, pretrained_results, model_results]
}

//update model/model data distribution on slider change.
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